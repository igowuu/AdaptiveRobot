# Motivation

AdaptiveRobot is built on principles that address common requirements in FRC robot architecture. I, the developer, have faced limitations and issues with commonly used FRC frameworks, so I wish to fix them for other teams to benefit.

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

## Component-Based Architecture

Components are the building basis of everything in a robot codebase.

Each component
- Publishes telemetry before executing
- Executes continuously, processing active requests each iteration
- Tracks its own health with fault thresholds
- Can be locked to bypass scheduling when needed

This separates the robot, actions, and specific components from each other for a cleaner codebase.

## Integrated Fault Management

Hardware failures require specific handling - logging diagnostics, disabling subsystems, and recording state. None of this should crash the robot by default, unless if its the intended action.

## Built-In Telemetry and Tuning

Some frameworks require manual NetworkTables integration for every parameter and sensor value. AdaptiveRobot includes telemetry and tuning as integrated features, similarly to Magicbot:

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
```java
Commands.runOnce(()->{
  // do something
}).andThen(Commands.run(() -> {
  // do looped something
})).until(condition).andThen(Commands.runOnce(() -> {
  // clean up
}));

Commands.run(co -> {
  // do something
  while(!condition) {
    // do looped something
    co.yield();
  }
  // clean up
});
```

Generally, FRC teams prefer the async mental module over the verbose commands2 alternative.

## Design Tradeoffs

AdaptiveRobot:
- **Automatic safety handling** over minimal framework responsibility
- **Priority-based arbitration** over exclusive resource locking
- **Built-in telemetry/tuning** over minimal overhead
- **Generator-based composition** over declarative command groups

This results in:
- More framework responsibilities (health tracking, fault handling, telemetry)
- Different mental model for resource management (requests vs commands, execute is the source of all arbitration)
- Smaller ecosystem compared to established frameworks

--- 
