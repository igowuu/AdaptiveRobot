# AdaptiveRobot Overview

AdaptiveRobot is a robot framework for FRC teams that want a cleaner way to write robot code without giving up safety, structure, or flexibility.

At a high level, AdaptiveRobot wraps the normal WPILib robot lifecycle and adds:

- a clear robot lifecycle
- automatic component / interface discovery
- request-based arbitration for shared outputs
- built-in telemetry and tunables
- built-in method profiling to diagnose loop overruns
- automatic fault handling and logging
- generator-based async actions for autonomous routines
- utilities to detect common hardware faults and log them to a file

The goal: let a student write robot code in normal Python, but make the framework handle the repetitive pieces, the safety checks, while allowing for as much flexibility as possible.

## Philosophy

AdaptiveRobot is built around a few ideas:

1. **Robot code should be organized by job, not by one large file.**
2. **Multiple systems should be able to ask for the same output safely.**
3. **Important values should be visible and adjustable at runtime.**
4. **Code should be profiled to detect loop overruns and other issues in the codebase.**
5. **Failures should be logged and handled, not hidden or ignored.**
6. **Autonomous should read like Python.**
7. **Users should not be restricted when coding by the scheduler.**
8. **Structure should not limit the users code or capability.**

## What AdaptiveRobot Gives You

### 1. Robot Lifecycle Management

AdaptiveRobot is built on top of `TimedRobot`. You override the `on...` methods in your robot class:

- `onRobotInit()` for startup work
- `onRobotPeriodic()` for code that should run in every mode
- `onDisabledInit()`, `onDisabledPeriodic()`, `onDisabledExit()`
- `onTeleopInit()`, `onTeleopPeriodic()`, `onTeleopExit()`
- `onAutonomousInit()`, `onAutonomousPeriodic()`, `onAutonomousExit()`
- `onTestInit()`, `onTestPeriodic()`, `onTestExit()`

The framework keeps the internal lifecycle methods unchangeable (@final) so the main scheduling logic stays consistent. Framework internals cannot be accidentally overriden by the user.

### 2. Adaptive Components and Interfaces

Robot code is meant to be split into interfaces and components. Interfaces are composable, so components only implement the behaviors they actually need:

- `Schedulable` for periodic execution and health tracking
- `TelemetryPublishable` for NetworkTables telemetry
- `TunablePublishable` for runtime-adjustable values
- `Faultable` for raising framework faults

`AdaptiveComponent` is the legacy class that combines these ideas into one object.

That means a drivetrain, intake, arm, shooter, or climber can all be written as their own class and still plug into the framework cleanly.

### 3. Auto-Discovery and Registration

Objects created in `onRobotInit()` are automatically discovered if they implement the supported interfaces and are attached to the robot object. Further, any object in onRobotInit will be recursively traced to detect all implementers of interfaces.

That significantly reduces boilerplate, but you can still manually register objects when you want full control.

### 4. Request-Based Arbitration

When more than one part of the robot wants to control the same output, AdaptiveRobot uses requests instead of letting the last source overwrite all others.

For example, teleop, autonomous, and safety logic can all request a drivetrain speed. The framework then resolves the active output based on priority and recency.

This is easier to reason about than a long chain of if statements, and it makes safety overrides straightforward. It allows for telemetry and reduces ambiguity upon user error.

Requests also support timeouts, so stale commands expire automatically if they are not refreshed.

### 5. Built-In Telemetry

Any class can publish its own telemetry by inheriting from `TelemetryPublishable`.

Examples of useful telemetry include:

- sensor readings
- motor speeds
- current mode
- subsystem state
- robot pose or other WPIStruct data

Telemetry is updated by the framework each loop, so your code can focus on what to report instead publishing it.

### 6. Built-In Tunables

Any class can publish its own tunables (and tunable PID objects) by inheriting from `TunablePublishable`.

Tunables let you change numbers like speeds, gains, offsets, and thresholds while the robot is running.

That means you can tune a drivetrain or arm without redeploying every time. The framework also supports tunable PID controllers so common control values can live in one place.

Tunables persist to disk on exit, so your values will not reset between redeploys.

### 7. Fault Handling and Logging

Robot code should not crash silently, nor simply write to terminal without any way to review it.

AdaptiveRobot includes fault handling so components, actions, and any other class inheriting from `Faultable` can report problems with a severity level:

- `WARNING` for minor issues
- `ERROR` for recoverable problems
- `CRITICAL` for problems that should disable the robot

The framework catches faults, logs them, and decides what should happen next. Critical faults can disable the robot cleanly instead of letting the program fail in a messy way.

### 8. Async Actions for Autonomous

AdaptiveRobot uses generator-based `AsyncAction` routines for autonomous and other long-running robot tasks.

You can write code that reads sequentially from top to bottom:

- wait for a moment
- move a mechanism
- run two things in parallel
- race one action against another

This makes autonomous routines easier for students to write and easier for mentors to review.

### 9. Safety and Health Tracking

Schedulable objects track whether they are healthy or faulted. When something goes wrong too many times (configurable for each `Subscheduler`), the framework can mark the object unhealthy and switch to faulted behavior.

This gives the robot a way to keep running safely even when one subsystem fails.

## 10. Profiling

Methods such as `execute()` and `publish_telemetry()` are automatically profiled by the framework. Additional methods can be profiled with the `@profile_method` decorator. Profiling data is periodically written to disk using the interval configured in `RobotConfig`.

## How It Fits Together

In a typical robot, the flow looks like this:

1. You create your robot class by inheriting from `AdaptiveRobot`.
2. In `onRobotInit()`, you create your components / interfaces / any other universal objects.
3. The framework discovers components and interfaces and connects them to telemetry, tunables, scheduling, and fault handling.
4. During each robot loop, the framework runs the schedulers and your periodic hooks.
5. If a fault happens, the framework logs it and reacts based on severity.
6. If you schedule an autonomous action, the action scheduler runs it until it finishes or is cancelled.

## Example Robot Structure

```python
from adaptive_robot.adaptive_robot import AdaptiveRobot


class MyRobot(AdaptiveRobot):
	def __init__(self) -> None:
		super().__init__()

	def onRobotInit(self) -> None:
		self.drivetrain = Drivetrain()
		self.intake = Intake()

	def onAutonomousInit(self) -> None:
		self.schedule_action(self.autonomous_routine(), "auto")
```

Your robot.py stays simple, and the scheduler handles everything else.

## Main Feature Areas

If you want the deeper docs for each part, AdaptiveRobot is split into feature guides:

- [Robot lifecycle and scheduling](./features/ROBOT_GUIDE.md)
- [Async actions and autonomous routines](./features/ACTIONS_GUIDE.md)
- [Request arbitration and shared outputs](./features/REQUESTS_GUIDE.md)
- [Fault handling and logging](./features/FAULTS_GUIDE.md)
- [Interfaces and component structure](./features/INTERFACE_GUIDE.md)
- [Profiling Guide](docs/features/PROFILING_GUIDE.md)
- [SysID Guide](docs/features/SYSID_GUIDE.md)
- [Utils Guide](docs/features/UTILS_GUIDE.md)

## Why Use It

AdaptiveRobot is meant to give teams a framework that feels more complete than a bare robot project, but still flexible enough to match real FRC code.

It helps you:

- keep robot code organized
- reduce boilerplate
- tune values without redeploying constantly
- log failures instead of guessing what went wrong
- write autonomous routines in a more natural style
- keep safety behavior in one place

-> Next: [Examples](../drivetrain_robot/README.md)
-> Next: [Features](features/ROBOT_GUIDE.md)
