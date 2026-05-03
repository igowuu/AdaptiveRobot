# Motivation

AdaptiveRobot is built on principles that address common requirements in FRC robot coding. I, the developer, have faced limitations and issues with commonly used FRC frameworks, so I wish to fix them for other teams to benefit.

## Request-Based Arbitration

Some schedulers use exclusive command ownership. One command claims a subsystem, and no other command can use it simultaneously. This is a valid structure, but it comes with its limitations. When multiple sources need the same resource (teleop, autonomous, safety systems), conflicts require interruption logic.

AdaptiveRobot uses request arbitration. Multiple sources submit requests with explicit priorities:

```python
drivetrain.speed.request(0.5, BasicPriority.TELEOP.value, "teleop")
drivetrain.speed.request(0.3, BasicPriority.AUTO.value, "auto")
drivetrain.speed.request(0.0, BasicPriority.SAFETY.value, "safety")

active = drivetrain.speed.resolve()  # Returns only the SAFETY request
```

So requests allow multiple different sources of output to request a component to produce an intended output at the same time without conflicts. 

## Interface-Based Architecture

Interfaces are the building basis of everything in a robot codebase.

TelemetryPublishable - Allows access to telemetry methods and lifecycle hooks.
TunablePublishable - Allows access to tunable and tunablePID methods.
Faultable - Allows a class to report its own Faults to the scheduler.
Schedulable - Allows a class to access all main lifecycle hooks (execute(), on_faulted_periodic(), on_enable(), etc), be locked, and track its health.

AdaptiveComponent - Interface that inherits from all of the above interfaces.

This allows for users to create customly structured classes without needing to work around AdaptiveComponent.

## Integrated Fault Management

Hardware failures require specific handling - logging diagnostics, disabling subsystems, and recording state. None of this should crash the robot by default, unless if its the intended action.

## Built-In Telemetry and Tuning

Some frameworks require manual NetworkTables integration for every parameter and sensor value. AdaptiveRobot includes telemetry and tuning as integrated features.:

```python
self.max_speed = self.tunable("Tunables/Speed", 1.0)
self.pid = self.tunablePID(kp=0.1, ki=0.0, kd=0.01)
self.publish_value("Motor/Speed", speed)
```

This reduces the boilerplate of manually writing and communicating with networktables.

## Request Timeouts

Requests specify a timeout duration. If a request is not resubmitted before its timeout expires, it is automatically removed. This prevents old requests from persisting:

```python
drivetrain.speed.request(0.5, TELEOP.value, "teleop", timeout=0.2)
```

Timeouts make request lifetime explicit and prevent requests from surviving beyond their intended duration.

---

## Async Actions

Asynchronous Actions allow for reduced boilerplate compared to more verbose autonomous systems. They allow for more pythonic syntax and allow for new students to pick up autonomous quicker.

For example:
```python
def autonomous_routine(robot: AdaptiveRobot) -> AsyncAction:
    # Everything in the routine will move sequentially unless wrapped
    yield from robot.drivetrain.move_forward(distance=5)
    yield from wait(0.5)
    yield from robot.intake.spin_up()
    
    # Run two things at the same time (until both finish)
    yield from parallel(
        robot.drivetrain.move_forward(distance=5),
        robot.intake.collect_pieces()
    )
  
    # Race two actions (continues once on finishes)
    yield from race(
        robot.arm.move_to_target(),
        wait(2.0)
    )
```

Generally, FRC teams prefer the async mental module over the verbose commands2 alternative.

## Design Tradeoffs

AdaptiveRobot:
- Automatic safety handling
- Priority-based arbitration
- Built-in telemetry/tuning
- Generator-based composition

This results in:
- More framework responsibilities (health tracking, fault handling, telemetry)
- Different mental model for resource management (requests and actions vs commands, execute is the source of all arbitration)
- Smaller ecosystem compared to Commands2

--- 
