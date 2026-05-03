# simple_component

This is the smallest example in the repository. It shows the minimum needed to use AdaptiveRobot with one mechanism and one request arbitrator (following the recommended structure).

1. `simple_io.py`
	- This file owns the motor hardware.
	- It shows the simplest possible IO layer: create a motor and expose one method to set voltage.

2. `simple_component.py`
	- This is the main component.
	- It inherits from `AdaptiveComponent`.
	- It creates one `RequestArbitrator` for percent output.
	- `request_percent()` lets other code ask for output with a priority and source name.
	- `execute()` resolves the winning request and sends voltage to the IO object.

## This example:

- The smallest useful AdaptiveRobot component
- A simple request-and-resolve flow
- Direct motor control through a an IO class
- How to separate subsystem logic from hardware access

-> Next: [Drivetrain Robot](../drivetrain_robot/README.md)
