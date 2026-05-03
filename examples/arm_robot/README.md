# arm_robot

This example is the next step after drivetrain_robot. It adds a closed-loop arm mechanism, tunable PID values, real and simulated IO selection, and a simple autonomous action.

## Read this example in order

1. robot.py
   - Creates the joystick and chooses real or simulated arm IO.
   - Creates the Arm component and ArmController.
   - Schedules a demo autonomous routine in onAutonomousInit().

2. arm_constants.py
   - Defines the arm limits, feedforward gains, PID gains, CAN IDs, and angle tolerance.

3. arm_io/io_base.py
   - Defines what any arm IO must provide: position, velocity, voltage, set_voltage(), and update().

4. arm_io/real_io.py and arm_io/simulated_io.py
   - real_io.py talks to Talon hardware and logs Talon faults.
   - simulated_io.py uses SingleJointedArmSim and Mechanism2d to run without hardware.
   - The rest of the robot code does not change when you swap these IO implementations.

5. arm.py
   - Actual subsystem logic.
   - Uses RequestArbitrator to select the active arm angle request.
   - Combines feedforward + PID each loop to move toward the target angle.
   - Publishes telemetry and uses lifecycle hooks for safe defaults.

6. arm_controller.py
   - Teleoperated control.
   - Maps joystick buttons to sticky target angles and submits TELEOP requests.
   - Keeping controller logic separate from mechanism logic is the key pattern.

7. autonomous/demo_auto.py
   - A simple async autonomous action.
   - Requests max angle, waits, then requests min angle.
   - Shows how to write loop-based autonomous behavior using AsyncAction.

## This example:

- Real vs simulated IO switching in robot init
- A closed-loop arm with feedforward and tunable PID
- Request-based arbitration between teleop and autonomous control
- Basic telemetry publishing for mechanism state
- A simple autonomous routine using AsyncAction
- Safe lifecycle defaults for enabled, disabled, and faulted states

-> Next: [Competition Tank Drive](../comp_tank_drive/README.md)
