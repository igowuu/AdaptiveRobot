# drivetrain_robot

This example shows a small but somewhat realistic drivetrain robot. It is meant to be the next step after the simple_component example: still easy to read, but now using a separate controller and a separate drivetrain component.

1. `robot.py`
	- Creates the drivetrain, drivetrain controller, and joystick.
	- AdaptiveRobot automatically discovers both objects because they inherit from framework interfaces.

2. `drivetrain.py`
	- The main subsystem logic goes here.
	- Owns the IO object.
	- Creates two `RequestArbitrator` instances, one for linear control and one for angular control.
	- Defines the safe default behavior and the lifecycle hooks for enabled, disabled, and faulted states.
	- Publishes basic telemetry and resolves the active requests during `execute()`.

3. `drivetrain_controller.py`
	- Reads the driver joystick during teleop.
	- Sends linear and angular percent requests to the drivetrain.
	- Keeps driver input separate from mechanism logic, which is the main pattern the example is teaching.

4. `drivetrain_io.py`
	- The communication between the component and hardware.
	- Owns the actual motor controllers and converts arcade drive inputs into motor output.
	- This file is intentionally small so the example stays easy to follow.

## This example:

- A single subsystem with a separate teleop controller
- Request-based control using priorities
- Basic lifecycle hooks for enabled, disabled, and faulted states
- Telemetry for debugging motor output
- A simple IO layer for direct hardware access

-> Next: [Arm Robot](../arm_robot/README.md)