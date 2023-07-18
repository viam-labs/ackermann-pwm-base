package pwmbase

import (
	"context"
	"github.com/pkg/errors"
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/utils"
	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/base"
	"go.viam.com/rdk/components/base/kinematicbase"
	"go.viam.com/rdk/components/servo"
	"go.viam.com/rdk/resource"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/operation"
)

var Model = resource.ModelNamespace("viam-labs").WithFamily("base").WithModel("ackermann-pwm-base")

const (
	defaultStoppedServoPosition = 90
	defaultMaxServoPosition = 180
	defaultMinServoPosition = 0
)

// AttrConfig is used for converting config attributes.
type Config struct {
	SteerServo string `json:"steer"`
	DriveServo string `json:"drive"`
	Radius float64 `json:"turning_radius_meters"`
	Wheelbase float64 `json:"wheelbase_mm"`
	MaxSpeedMPS float64 `json:"max_speed_meters_per_second"`
	InvertSteer bool `json:"invert_steer,omitempty"`
	InvertDrive bool `json:"invert_drive,omitempty"`
}

// Validate ensures all parts of the config are valid.
func (config *Config) Validate(path string) ([]string, error) {
	var deps []string
	if config.SteerServo == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "steer")
	}
	if config.DriveServo == "" {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "drive")
	}
	if config.Radius == math.NaN() {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "turning_radius_meters")
	}
	if config.Wheelbase == math.NaN() {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "wheelbase_mm")
	}
	if config.MaxSpeedMPS == math.NaN() {
		return nil, utils.NewConfigValidationFieldRequiredError(path, "max_speed_meters_per_second")
	}
	
	deps = append(deps, config.SteerServo, config.DriveServo)

	return deps, nil
}

func init() {
	resource.RegisterComponent(
		base.API,
		Model,
		resource.Registration[base.Base, *Config]{
			Constructor: func(
				ctx context.Context,
				deps resource.Dependencies,
				conf resource.Config,
				logger golog.Logger,
			) (base.Base, error) {
				return newBase(ctx, deps, conf.ResourceName(), conf, logger)
		},
	})
}

func newBase(
	ctx context.Context,
	deps resource.Dependencies,
	name resource.Name,
	attr resource.Config,
	logger golog.Logger,
) (base.Base, error) {
	ab := &ackermannPwmBase{
		Named:    name.AsNamed(),
		logger: logger,
		
		// TODO: make these configurable
		neutralServoPos: defaultStoppedServoPosition,
		maxServoPos: defaultMaxServoPosition,
		minServoPos: defaultMinServoPosition,
	}
	

	if err := ab.Reconfigure(ctx, deps, attr); err != nil {
		return nil, err
	}
	
	return ab, nil
}

type ackermannPwmBase struct {
	resource.Named
	geometries           []spatialmath.Geometry

	steer     servo.Servo
	drive     servo.Servo
	
	maxSpeedMPS float64
	turnRadMeters float64
	
	wheelbaseMm float64
	
	neutralServoPos uint32
	maxServoPos uint32
	minServoPos uint32
	
	invertSteer bool
	invertDrive bool
	
	maxSteerAngleDeg float64

	opMgr  operation.SingleOperationManager
	logger golog.Logger

	mu   sync.Mutex
	name string
}

// Reconfigure reconfigures the base atomically and in place.
func (ab *ackermannPwmBase) Reconfigure(ctx context.Context, deps resource.Dependencies, conf resource.Config) error {
	ab.mu.Lock()
	defer ab.mu.Unlock()

	geometries, err := kinematicbase.CollisionGeometry(conf.Frame)
	if err != nil {
		ab.logger.Warnf("base %v %s", ab.Name(), err.Error())
	}
	ab.geometries = geometries

	newConf, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return err
	}
	
	ab.turnRadMeters = newConf.Radius
	ab.maxSpeedMPS = newConf.MaxSpeedMPS
	
	ab.wheelbaseMm = newConf.Wheelbase
	
	// calculate steer angle from parameters https://www.vcrashusa.com/kb-vc-article99
	ab.maxSteerAngleDeg = wheelAngleFromRadiusAndWheelbase(ab.turnRadMeters, ab.wheelbaseMm)

	updateMotors := func(curr servo.Servo, fromConfig string, whichMotor string) (servo.Servo, error) {
		var newServo servo.Servo
		select {
		case <-ctx.Done():
			return newServo, fmt.Errorf("timeout while rebuilding %s", ab.Name())
		default:
		}
		serv, err := resource.FromDependencies[servo.Servo](deps, resource.NewName(servo.API, fromConfig))
		if err != nil {
			return newServo, errors.Wrapf(err, "no %s motor named (%s)", whichMotor, fromConfig)
		}
		return serv, nil
	}

	steer, err := updateMotors(ab.steer, newConf.SteerServo, "steer")
	ab.steer = steer
	if err != nil {
		return ab.doDrive(ctx, ab.neutralServoPos)
	}
	drive, err := updateMotors(ab.drive, newConf.DriveServo, "drive")
	ab.drive = drive
	if err != nil {
		return err
	}
	
	if newConf.InvertSteer {
		ab.invertSteer = true
	}
	if newConf.InvertDrive {
		ab.invertDrive = true
	}

	return nil
}

// Spin commands a base to turn about its center at a angular speed and for a specific angle.
func (ab *ackermannPwmBase) Spin(ctx context.Context, angleDeg, degsPerSec float64, extra map[string]interface{}) error {
	return errors.New("ackermann base does not support Spin")
}

// MoveStraight commands a base to drive forward or backwards  at a linear speed and for a specific distance.
func (ab *ackermannPwmBase) MoveStraight(ctx context.Context, distanceMm int, mmPerSec float64, extra map[string]interface{}) error {
	ctx, done := ab.opMgr.New(ctx)
	defer done()
	ab.logger.Infof("received a MoveStraight with distanceMM:%d, mmPerSec:%.2f", distanceMm, mmPerSec)
	if mmPerSec / 1000. > ab.maxSpeedMPS {
		return fmt.Errorf("requested speed %f is greater than maximum base speed %f", mmPerSec / 1000., ab.maxSpeedMPS)
	}
	if mmPerSec == 0 {
		return errors.New("cannot move base at 0 mm per sec")
	}
	err := ab.doSteer(ctx, ab.neutralServoPos)
	if err != nil {
		return err
	}
	
	driveSec := float64(distanceMm) / mmPerSec
	driveVal := ab.proportionToServo((mmPerSec/1000.)/ab.maxSpeedMPS)
	runTime := time.Duration(math.Abs(driveSec)) * time.Second
	err = ab.doDrive(ctx, driveVal)
	if err != nil {
		return err
	}
	utils.SelectContextOrWait(ctx, runTime)
	return ab.Stop(ctx, nil)
}

// SetVelocity commands the base to move at the input linear and angular velocities.
func (ab *ackermannPwmBase) SetVelocity(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	ab.opMgr.CancelRunning(ctx)

	ab.logger.Infof(
		"received a SetVelocity with linear.X: %.2f, linear.Y: %.2f linear.Z: %.2f(mmPerSec),"+
			" angular.X: %.2f, angular.Y: %.2f, angular.Z: %.2f",
		linear.X, linear.Y, linear.Z, angular.X, angular.Y, angular.Z)

	if linear.Y / 1000. > ab.maxSpeedMPS {
		return fmt.Errorf("requested speed %f is greater than maximum base speed %f", linear.Y / 1000., ab.maxSpeedMPS)
	}
	
	maxAngVelRadPS := (linear.Y / 1000.) / ab.turnRadMeters
	if math.Abs(rdkutils.RadToDeg(maxAngVelRadPS)) < math.Abs(angular.Z) {
		return fmt.Errorf(
			"at requested speed of %f mm/s, a base with turning radius %f mm can turn at most %f degrees per second",
			linear.Y,
			ab.turnRadMeters * 1000,
			math.Abs(rdkutils.RadToDeg(maxAngVelRadPS)),
		)
	}
	
	if angular.Z != 0 {
		// If we are here then requested lin/ang velocities are valid
		// Turning radius and steering angle have an exponential relationship so we have to do fancy math now
		desiredTurnRad := math.Abs(linear.Y / 1000.) / rdkutils.DegToRad(math.Abs(angular.Z))
		turnAngle := math.Copysign(wheelAngleFromRadiusAndWheelbase(desiredTurnRad, ab.wheelbaseMm), angular.Z)
		if linear.Y < 0 {
			turnAngle *= -1
		}
		
		err := ab.doSteer(ctx, ab.proportionToServo(turnAngle / ab.maxSteerAngleDeg))
		if err != nil {
			return err
		}
	} else {
		err := ab.doSteer(ctx, ab.neutralServoPos)
		if err != nil {
			return err
		}
	}
	return ab.doDrive(ctx, ab.proportionToServo((linear.Y / 1000) / ab.maxSpeedMPS))
}

// SetPower commands the base motors to run at powers corresponding to input linear and angular powers.
func (ab *ackermannPwmBase) SetPower(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	ab.opMgr.CancelRunning(ctx)

	ab.logger.Infof(
		"received a SetPower with linear.X: %.2f, linear.Y: %.2f linear.Z: %.2f,"+
			" angular.X: %.2f, angular.Y: %.2f, angular.Z: %.2f",
		linear.X, linear.Y, linear.Z, angular.X, angular.Y, angular.Z)

	err := ab.doSteer(ctx, ab.proportionToServo(angular.Z))
	if err != nil {
		return err
	}
	return ab.doDrive(ctx, ab.proportionToServo(linear.Y))
}

// Stop commands the base to stop moving.
func (ab *ackermannPwmBase) Stop(ctx context.Context, extra map[string]interface{}) error {
	return ab.doDrive(ctx, ab.neutralServoPos)
}

func (ab *ackermannPwmBase) IsMoving(ctx context.Context) (bool, error) {
	position, err := ab.drive.Position(ctx, nil)
	if err != nil {
		return false, err
	}
	if position != ab.neutralServoPos {
		return true, err
	}
	return false, nil
}

// Close is called from the client to close the instance of the wheeledBase.
func (ab *ackermannPwmBase) Close(ctx context.Context) error {
	return ab.Stop(ctx, nil)
}

func (ab *ackermannPwmBase) Properties(ctx context.Context, extra map[string]interface{}) (base.Properties, error) {
	return base.Properties{
		TurningRadiusMeters: ab.turnRadMeters,
	}, nil
}

func (ab *ackermannPwmBase) Geometries(ctx context.Context) ([]spatialmath.Geometry, error) {
	return ab.geometries, nil
}

func (ab *ackermannPwmBase) doDrive(ctx context.Context, val uint32) error {
	if ab.invertDrive {
		if val > ab.neutralServoPos {
			val = ab.neutralServoPos - (val - ab.neutralServoPos)
		} else  {
			val = ab.neutralServoPos + (ab.neutralServoPos - val)
		}
	}
	return ab.drive.Move(ctx, val, nil)
}

func (ab *ackermannPwmBase) doSteer(ctx context.Context, val uint32) error {
	if ab.invertSteer {
		if val > ab.neutralServoPos {
			val = ab.neutralServoPos - (val - ab.neutralServoPos)
		} else  {
			val = ab.neutralServoPos + (ab.neutralServoPos - val)
		}
	}
	return ab.steer.Move(ctx, val, nil)
}

// transforms a proportional value [-1 to 1] to the appropriate servo degree number
func (ab *ackermannPwmBase) proportionToServo(val float64) uint32 {
	if val > 1 {
		val = 1.
	}
	if val < -1 {
		val = -1.
	}
	
	altVal := val * float64(ab.maxServoPos - ab.neutralServoPos)
	
	return uint32(float64(ab.neutralServoPos) + altVal)
}

func wheelAngleFromRadiusAndWheelbase(turnRadMeters, wheelbaseMm float64) float64 {
	return rdkutils.RadToDeg(math.Atan2(
		math.Sqrt(math.Pow(wheelbaseMm, 2) / (math.Pow(turnRadMeters * 1000, 2) - math.Pow(wheelbaseMm, 2))),
		wheelbaseMm,
	))
}
