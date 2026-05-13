# Request Arbitration

## Overview

The request-based arbitration system solves the problem: what happens when multiple parts of your robot want to control the same thing at the same time?

Imagine your drivetrain getting requests from the driver (teleop), an autonomous routine (auto), and a safety system. Who wins? RequestArbitrator handles this by letting all three submit requests with priorities, then picking the winner automatically each loop.

## Quick Start

Create a `RequestArbitrator` for each output you want to control, then submit requests from different sources:

```python
from adaptive_robot import AdaptiveComponent, RequestArbitrator, BasicPriority

class Drivetrain(AdaptiveComponent):
    def __init__(self) -> None:
        super().__init__()
        self.speed_controller = RequestArbitrator()  # Arbitrate speed requests
    
    def request_speed(self, speed: float, priority: BasicPriority, source: str) -> None:
        """Request a speed from any source."""
        self.speed_controller.request(speed, priority.value, source)
    
    def execute(self) -> None:
        # Always runs the highest-priority request
        active = self.speed_controller.resolve()
        self.motor.set(active.value)
```

Multiple sources can request at the same time:

```python
# Teleop requests forward
drivetrain.request_speed(0.5, BasicPriority.TELEOP, "teleop")

# Autonomous also requests forward
drivetrain.request_speed(0.3, BasicPriority.AUTO, "auto")

# Safety system requests stop
drivetrain.request_speed(0.0, BasicPriority.SAFETY, "safety")

# Safety wins (priority 3 > auto priority 2 > teleop priority 1)
```

---

## Core Concepts

### Priority: Who Wins?

When `resolve()` is called, the arbitrator picks based on:

1. **Highest priority number wins** - SAFETY (3) beats AUTO (2) beats TELEOP (1)
2. **If tied priority, most recent timestamp wins** - Newer requests override older ones at the same priority
3. **If tie-breaking still needed, most recently inserted wins** - The last one submitted in the same iteration

### Timeout: Auto-Expire Old Requests

Each request has a timeout. If a request isn't resubmitted before time runs out, it's automatically removed. This is a safety feature so your robot won't retain old commands.

```python
# Request times out in 0.2 seconds if not resubmitted
drivetrain.request_speed(0.5, BasicPriority.TELEOP, "teleop", timeout=0.2)

# If the joystick input stops, the request expires automatically
# Next resolve() will use a lower-priority request or the default
```

All requests can use different timeouts if desired.

### Default Request: The Fallback

When no active requests exist, the arbitrator returns a default request (usually 0.0 at priority -1). You set this when creating the arbitrator:

```python
speed_controller = RequestArbitrator(
    default_value=0.0,        # Safe default
    default_priority=-1,      # Lower than any real request
    default_source="default"
)
```

---

## API Reference

### Creating an Arbitrator

```python
controller = RequestArbitrator(
    default_value=0.0,        # Returned when no requests are active
    default_priority=-1,      # Must be different from all submitted priorities
    default_source="default"  # Name of the default request
)
```

### Submitting Requests

```python
# Submit a request
controller.request(
    value=0.5,                              # The commanded value
    priority=BasicPriority.TELEOP.value,   # 1 (TELEOP), 2 (AUTO), 3 (SAFETY)
    source="teleop_left_stick",             # Unique name (alphanumeric + underscores, max 50 chars)
    timeout=0.2                             # How long it stays active (0 < timeout <= 10.0)
)

# If you submit again from the same source, the old one is replaced
# This is useful for continuous inputs (like joysticks)
```

**Constraints:**
- `source` must be unique, alphanumeric + underscores only, max 50 characters
- `priority` must NOT equal the default priority
- `timeout` must be between 0 (exclusive) and 10.0 seconds

### Resolving (Getting the Winner)

```python
# Get the active request
active = controller.resolve()

print(f"Value: {active.value}")      # The commanded value
print(f"Priority: {active.priority}") # Its priority level
print(f"Source: {active.source}")    # Where it came from
```

Automatically removes timed-out requests before deciding. If nothing is active, returns the default.

### Clearing & Enabling

```python
# Remove all active requests (falls back to default)
controller.clear()

# Disable the controller (always returns default)
controller.set_enabled(False)

# Re-enable later
controller.set_enabled(True)

# Check state
if controller.is_enabled():
    print("Controller is active")
```

### Introspection

```python
# How many requests are currently active?
count = controller.get_pending_request_count()

# What's the default request?
default = controller.get_default_request()

# What was the last resolved request?
last = controller.last_request
print(f"Last source: {last.source}")
```

---

## Usage Examples

Patterns from the competition example (`examples/comp_tank_drive`).

### Example 1: Simple Drivetrain Arbitration

Create request methods on your subsystem, then call them from controllers:

```python
class Drivetrain(AdaptiveComponent):
    def __init__(self, io):
        super().__init__()
        self.io = io
        self.linear_velocity_controller = RequestArbitrator()
        self.angular_velocity_controller = RequestArbitrator()
    
    def request_linear_velocity(self, velocity, priority, source):
        """Public method for requesting linear speed."""
        self.linear_velocity_controller.request(velocity, priority.value, source)
    
    def request_angular_velocity(self, velocity, priority, source):
        """Public method for requesting angular speed."""
        self.angular_velocity_controller.request(velocity, priority.value, source)
    
    def execute(self) -> None:
        # Resolve and apply the highest-priority requests
        linear = self.linear_velocity_controller.resolve().value
        angular = self.angular_velocity_controller.resolve().value
        
        # Command hardware with resolved values
        wheel_speeds = self.kinematics.toWheelSpeeds(ChassisSpeeds(linear, 0, angular))
        self.io.set_left_voltage(wheel_speeds.left)
        self.io.set_right_voltage(wheel_speeds.right)
```

### Example 2: Teleop Controller Requesting

Controllers submit requests on behalf of drivers. Each request is tagged with the source:

```python
class DrivetrainController(Schedulable):
    def __init__(self, drivetrain: Drivetrain, controller: Joystick):
        self.drivetrain = drivetrain
        self.controller = controller
    
    def execute(self) -> None:
        if not RobotState.isTeleop():
            return
        
        # Read joystick
        linear = self.controller.getRawAxis(0) * MAX_SPEED
        angular = self.controller.getRawAxis(1) * MAX_ANGULAR
        
        # Request both
        self.drivetrain.request_linear_velocity(
            linear,
            BasicPriority.TELEOP,
            "teleop_driver"
        )
        self.drivetrain.request_angular_velocity(
            angular,
            BasicPriority.TELEOP,
            "teleop_driver"
        )
```

### Example 3: Safety Override (Multiple Priorities)

Different systems request at different priorities. Safety always wins:

```python
def onTeleopPeriodic(self) -> None:
    # Teleop makes a request
    self.drivetrain.request_linear_velocity(
        joystick_input,
        BasicPriority.TELEOP,
        "teleop"
    )
    
    # Safety system can override
    if self.too_close_to_wall():
        self.drivetrain.request_linear_velocity(
            0.0,  # Stop
            BasicPriority.SAFETY,
            "wall_bumper"
        )

def execute(self) -> None:
    # resolve() picks the SAFETY request automatically
    active = self.drivetrain.linear_velocity_controller.resolve()
    self.motor.set(active.value)
```

### Example 4: Autonomous Request with Different Timeout

Auto routines use different timeouts than teleop:

```python
def onAutonomousInit(self) -> None:
    # Auto request times out slower (routine runs longer)
    self.drivetrain.request_linear_velocity(
        0.5,
        BasicPriority.AUTO,
        "auto_drive",
        timeout=1.0  # Longer timeout for autonomous
    )

def onTeleopPeriodic(self) -> None:
    # Teleop requests are quick (driver input might stop)
    self.drivetrain.request_linear_velocity(
        joystick.getRawAxis(0),
        BasicPriority.TELEOP,
        "teleop_driver",
        timeout=0.2  # Short timeout
    )
```

### Example 5: Multiple Subsystems with Separate Arbitrators

Each subsystem (drivetrain, intake, shooter) has its own arbitrators:

```python
class Intake(AdaptiveComponent):
    def __init__(self, io):
        super().__init__()
        self.io = io
        self.percent_controller = RequestArbitrator()  # Separate from drivetrain
    
    def request_percent(self, percent, priority, source):
        self.percent_controller.request(percent, priority.value, source)

class Shooter(AdaptiveComponent):
    def __init__(self, io):
        super().__init__()
        self.io = io
        self.velocity_controller = RequestArbitrator()  # Separate from everything else
    
    def request_velocity(self, velocity, priority, source):
        self.velocity_controller.request(velocity, priority.value, source)

# Each subsystem resolves independently
intake_request = intake.percent_controller.resolve()
shooter_request = shooter.velocity_controller.resolve()

intake.io.set_voltage(intake_request.value)
shooter.io.set_voltage(shooter_request.value)
```

---

## Things to Remember

- **Priority must be different from default.** By default, don't use -1 as a priority (you will get an error if the default priority is used elsewhere). Use positive integers (1, 2, 3, etc.)
- **Timeout can't be 0 or greater than 10.** This prevents mistakes like timeout=0 (request expires immediately) or timeout=1000 (old requests hang around forever).
- **Source names are strict.** Alphanumeric + underscores only. No spaces, no dashes, no special chars. The framework validates this.
- **Requests are per-source.** If you call `request(..., source="teleop")` twice, the second call replaces the first. You can't have two requests from the same source active at once.
- **resolve() always runs.** Call it every loop in `execute()`. Each call removes timed-out requests automatically.
- **Priority wins over recency.** A higher-priority request always beats a newer lower-priority request. Don't rely on "newest" if priorities differ.
- **The default never times out.** The default request has an infinite timeout (`timeout=inf`), so it's always there as a fallback.
