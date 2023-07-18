# ackermann-pwm-base
A module for a ackermann-steered base that controls steering and driving via pwm servos

## Usage

### 1. Build binary

If you clone this repository to the target environment where you run your Viam robot, then you can build a binary named `ackermann` with:

```
go build -o ackermann
```

Alternatively, if you want to build a binary for a different target environment, please use the [viam canon tool](https://github.com/viamrobotics/canon).

### 2. Add to robot configuration

Copy the binary to the robot (system where viam-server is running) and add the following to your configuration:

```
  ...
  "modules": [
    ...,
    {
      "executable_path": "<path_to_binary>",
      "name": "ackermann-module"
    },
    ...,
  ],
  "components": [
    ...,
    {
      "name": "anything",
      "type": "base",
      "model": "viam-labs:base:ackermann-pwm-base",
      "attributes": {
        "steer": "steerServo",
        "drive": "driveServo",
        "turning_radius_meters": 0.2,
        "wheelbase_mm": 300,
        "max_speed_meters_per_second": 2.5
      }
    },
    ...,
  ],
  ...
```

Explanation of required configuration parameters:

"steer": The name of the servo used for steering

"drive": The name of the servo used for driving

"turning_radius_meters": The turning radius of the base as measured from its centerpoint, in meters

"wheelbase_mm": Distance from the front wheels to the rear wheels

"max_speed_meters_per_second": The speed at which the robot will travel if driven at 100%.

The following are optional configuration parameters:

"invert_steer": Set this to `true` to swap left/right steering direction

"invert_drive": Set this to `true` to swap forward/reverse driving directions

For more information on how to configure modular components, [see this example](https://docs.viam.com/services/slam/run-slam-cartographer/#step-1-add-your-rdiplar-as-a-modular-component).
