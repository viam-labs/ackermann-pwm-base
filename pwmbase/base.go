// Package nau7802module implements a nau7802 sensor for load cell amplification
// datasheet can be found at: https://cdn.sparkfun.com/assets/e/c/8/7/6/NAU7802_Data_Sheet_V1.7.pdf
// example repo: https://github.com/sparkfun/SparkFun_Qwiic_Scale_NAU7802_Arduino_Library
package nau7802module

import (
	"context"
	"encoding/hex"
	"github.com/pkg/errors"
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/edaniels/golog"
	"go.viam.com/utils"
	"go.uber.org/multierr"
	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/base"
	"go.viam.com/rdk/components/base/kinematicbase"
	"go.viam.com/rdk/components/servo"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/operation"
)

var Model = resource.ModelNamespace("viam-labs").WithFamily("base").WithModel("ackermann-pwm-base")

// AttrConfig is used for converting config attributes.
type Config struct {
	SteerServo string `json:"steer"`
	DriveServo string `json:"drive"`
	Radius float64 `json:"turning_radius_meters"`
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
				newConf, err := resource.NativeConfig[*Config](conf)
				if err != nil {
					return nil, err
				}
				return newBase(ctx, deps, conf.ResourceName(), newConf, logger)
		},
	})
}

func newBase(
	ctx context.Context,
	deps resource.Dependencies,
	name resource.Name,
	attr *Config,
	logger golog.Logger,
) (base.Base, error) {
	b := &ackermannPwmBase{
		Named:    name.AsNamed(),
		logger: logger,
	}
	

	return b, nil
}

type ackermannPwmBase struct {
	resource.Named
	geometries           []spatialmath.Geometry

	steer     servo.Servo
	drive     servo.Servo

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
		return err
	}
	drive, err := updateMotors(ab.drive, newConf.DriveServo, "drive")
	ab.drive = drive
	if err != nil {
		return err
	}

	ab.allMotors = append(ab.allMotors, ab.left...)
	ab.allMotors = append(ab.allMotors, ab.right...)

	if ab.widthMm != newConf.WidthMM {
		ab.widthMm = newConf.WidthMM
	}

	if ab.wheelCircumferenceMm != newConf.WheelCircumferenceMM {
		ab.wheelCircumferenceMm = newConf.WheelCircumferenceMM
	}

	return nil
}

// createWheeledBase returns a new wheeled base defined by the given config.
func createWheeledBase(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger golog.Logger,
) (base.Base, error) {
	newConf, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}

	ab := wheeledBase{
		Named:                conf.ResourceName().AsNamed(),
		widthMm:              newConf.WidthMM,
		wheelCircumferenceMm: newConf.WheelCircumferenceMM,
		spinSlipFactor:       newConf.SpinSlipFactor,
		logger:               logger,
		name:                 conf.Name,
	}

	if err := ab.Reconfigure(ctx, deps, conf); err != nil {
		return nil, err
	}

	if len(newConf.MovementSensor) != 0 {
		sb := sensorBase{wBase: &ab, logger: logger, Named: conf.ResourceName().AsNamed()}
		if err := sb.Reconfigure(ctx, deps, conf); err != nil {
			return nil, err
		}
		return &sb, nil
	}

	return &ab, nil
}

// Spin commands a base to turn about its center at a angular speed and for a specific angle.
func (ab *ackermannPwmBase) Spin(ctx context.Context, angleDeg, degsPerSec float64, extra map[string]interface{}) error {
	return errors.New("ackermann base does not support Spin")
}

// MoveStraight commands a base to drive forward or backwards  at a linear speed and for a specific distance.
func (ab *ackermannPwmBase) MoveStraight(ctx context.Context, distanceMm int, mmPerSec float64, extra map[string]interface{}) error {
	ctx, done := ab.opMgr.New(ctx)
	defer done()
	ab.logger.Debugf("received a MoveStraight with distanceMM:%d, mmPerSec:%.2f", distanceMm, mmPerSec)

	// Stop the motors if the speed or distance are 0
	if math.Abs(mmPerSec) < 0.0001 || distanceMm == 0 {
		err := ab.Stop(ctx, nil)
		if err != nil {
			return errors.Errorf("error when trying to move straight at a speed and/or distance of 0: %v", err)
		}
		return err
	}

	// Straight math
	rpm, rotations := ab.straightDistanceToMotorInputs(distanceMm, mmPerSec)

	return ab.runAll(ctx, rpm, rotations, rpm, rotations)
}

// runAll executes motor commands in parallel for left and right motors,
// with specified speeds and rotations and stops the base if an error occurs.
func (ab *ackermannPwmBase) runAll(ctx context.Context, leftRPM, leftRotations, rightRPM, rightRotations float64) error {
	fs := []rdkutils.SimpleFunc{}

	for _, m := range ab.left {
		fs = append(fs, func(ctx context.Context) error { return m.GoFor(ctx, leftRPM, leftRotations, nil) })
	}

	for _, m := range ab.right {
		fs = append(fs, func(ctx context.Context) error { return m.GoFor(ctx, rightRPM, rightRotations, nil) })
	}

	if _, err := rdkutils.RunInParallel(ctx, fs); err != nil {
		return multierr.Combine(err, ab.Stop(ctx, nil))
	}
	return nil
}

// SetVelocity commands the base to move at the input linear and angular velocities.
func (ab *ackermannPwmBase) SetVelocity(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	ab.opMgr.CancelRunning(ctx)

	ab.logger.Debugf(
		"received a SetVelocity with linear.X: %.2f, linear.Y: %.2f linear.Z: %.2f(mmPerSec),"+
			" angular.X: %.2f, angular.Y: %.2f, angular.Z: %.2f",
		linear.X, linear.Y, linear.Z, angular.X, angular.Y, angular.Z)

	l, r := ab.velocityMath(linear.Y, angular.Z)

	return ab.runAll(ctx, l, 0, r, 0)
}

// SetPower commands the base motors to run at powers corresponding to input linear and angular powers.
func (ab *ackermannPwmBase) SetPower(ctx context.Context, linear, angular r3.Vector, extra map[string]interface{}) error {
	ab.opMgr.CancelRunning(ctx)

	ab.logger.Debugf(
		"received a SetPower with linear.X: %.2f, linear.Y: %.2f linear.Z: %.2f,"+
			" angular.X: %.2f, angular.Y: %.2f, angular.Z: %.2f",
		linear.X, linear.Y, linear.Z, angular.X, angular.Y, angular.Z)

	lPower, rPower := ab.differentialDrive(linear.Y, angular.Z)

	// Send motor commands
	var err error
	for _, m := range ab.left {
		err = multierr.Combine(err, m.SetPower(ctx, lPower, extra))
	}

	for _, m := range ab.right {
		err = multierr.Combine(err, m.SetPower(ctx, rPower, extra))
	}

	if err != nil {
		return multierr.Combine(err, ab.Stop(ctx, nil))
	}

	return nil
}

// returns rpm, revolutions for a spin motion.
func (ab *ackermannPwmBase) spinMath(angleDeg, degsPerSec float64) (float64, float64) {
	wheelTravel := ab.spinSlipFactor * float64(ab.widthMm) * math.Pi * (angleDeg / 360.0)
	revolutions := wheelTravel / float64(ab.wheelCircumferenceMm)
	revolutions = math.Abs(revolutions)

	// RPM = revolutions (unit) * deg/sec * (1 rot / 2pi deg) * (60 sec / 1 min) = rot/min
	// RPM = (revolutions (unit) / angleDeg) * degPerSec * 60
	rpm := (revolutions / angleDeg) * degsPerSec * 60

	return rpm, revolutions
}

// calcualtes wheel rpms from overall base linear and angular movement velocity inputs.
func (ab *ackermannPwmBase) velocityMath(mmPerSec, degsPerSec float64) (float64, float64) {
	// Base calculations
	v := mmPerSec
	r := float64(ab.wheelCircumferenceMm) / (2.0 * math.Pi)
	l := float64(ab.widthMm)

	w0 := degsPerSec / 180 * math.Pi
	wL := (v / r) - (l * w0 / (2 * r))
	wR := (v / r) + (l * w0 / (2 * r))

	// RPM = revolutions (unit) * deg/sec * (1 rot / 2pi deg) * (60 sec / 1 min) = rot/min
	rpmL := (wL / (2 * math.Pi)) * 60
	rpmR := (wR / (2 * math.Pi)) * 60

	return rpmL, rpmR
}

// calculates the motor revolutions and speeds that correspond to the required distance and linear speeds.
func (ab *ackermannPwmBase) straightDistanceToMotorInputs(distanceMm int, mmPerSec float64) (float64, float64) {
	// takes in base speed and distance to calculate motor rpm and total rotations
	rotations := float64(distanceMm) / float64(ab.wheelCircumferenceMm)

	rotationsPerSec := mmPerSec / float64(ab.wheelCircumferenceMm)
	rpm := 60 * rotationsPerSec

	return rpm, rotations
}

// Stop commands the base to stop moving.
func (ab *ackermannPwmBase) Stop(ctx context.Context, extra map[string]interface{}) error {
	var err error
	for _, m := range ab.allMotors {
		err = multierr.Combine(err, m.Stop(ctx, extra))
	}
	return err
}

func (ab *ackermannPwmBase) IsMoving(ctx context.Context) (bool, error) {
	for _, m := range ab.allMotors {
		isMoving, _, err := m.IsPowered(ctx, nil)
		if err != nil {
			return false, err
		}
		if isMoving {
			return true, err
		}
	}
	return false, nil
}

// Close is called from the client to close the instance of the wheeledBase.
func (ab *ackermannPwmBase) Close(ctx context.Context) error {
	return ab.Stop(ctx, nil)
}

func (ab *ackermannPwmBase) Properties(ctx context.Context, extra map[string]interface{}) (base.Properties, error) {
	return base.Properties{
		TurningRadiusMeters: 0.0,
		WidthMeters:         float64(ab.widthMm) * 0.001, // convert to meters from mm
	}, nil
}

func (ab *ackermannPwmBase) Geometries(ctx context.Context) ([]spatialmath.Geometry, error) {
	return ab.geometries, nil
}
