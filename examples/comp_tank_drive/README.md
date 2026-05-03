# comp_tank_drive

This example is the most complete robot in the repository. It is meant to show what a real competition bot can look like when AdaptiveRobot is used for all of the major robot features.

It is intentionally more advanced than the basic examples, so this README stays high level. The goal is to help you understand where to look first, not to explain every line of code.

1. `robot.py`
	- Creates the joystick, chooses real or simulated IO, builds the components, and registers the controllers.
	- Also starts autonomous actions.

2. `components/`
	- Each mechanism has its own folder.
	- The drivetrain, intake, intake arm, shooter, and game simulation are split into separate subsystems.
	- Each subsystem typically has a component file, a controller file, constants, and real/sim IO.

3. `actions/`
	- Autonomous routines live here.
	- `taxi_drive.py` starts the autonomous example.
	- `follow_trajectory.py` shows how the robot follows a Choreo trajectory.

4. `fuel_sim/`
	- Ssimulation support code for game pieces and field behavior.
	- One of the more advanced parts of the example and is mainly here to show what a richer sim setup can look like.

5. `faults/`
	- Fault logs from testing are stored here.
	- Allows for reviewing issues during a match / after a match per session.

## This example:

- Real and simulated IO selection in one robot class
- Multiple subsystems with separate controllers
- Request arbitration for teleop, autonomous, and safety behavior
- Telemetry publishing and live tuning
- Autonomous action scheduling
- Fault logging and recovery patterns
- A more realistic competition workflow with drivetrain, intake, arm, and shooter
