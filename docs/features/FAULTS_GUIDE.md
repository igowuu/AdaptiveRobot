# Fault Handling

## Overview

Your robot runs in an unpredictable environment, and its guarenteed that it will fail at some point in time.

AdaptiveRobot's fault system lets you detect problems, log them, and respond gracefully. When something goes wrong, you raise a **fault**. The framework catches it, logs it to a file for later analysis, and decides what happens next based on the fault's severity. You have control over how your robot responds to any issues.

## Quick Start

Detect a problem and raise a fault:

```python
from adaptive_robot import AdaptiveComponent, Faultable, FaultSeverity

class Intake(AdaptiveComponent):
    def __init__(self, io) -> None:
        super().__init__()
        self.io = io
    
    def execute(self) -> None:
        try:
            # Try to read a sensor
            distance = self.io.get_distance()
            if distance < 0:  # Bad sensor reading
                self.raise_fault(
                    self,
                    FaultSeverity.ERROR,
                    "Distance sensor returned negative value"
                )
        except Exception as e:
            # Hardware failure (timeout, CAN error, etc)
            self.raise_fault(
                self,
                FaultSeverity.ERROR,
                "Distance sensor communication failed",
                exception=e
            )
        
        # If no exception, command hardware normally
        self.io.set_speed(0.5)
```

That's it. When a fault is raised:
1. The framework logs it to a file (for debugging later)
2. The component becomes **unhealthy** (after 10 consecutive faults)
3. Your component's `execute()` stops running; instead, `on_faulted_periodic()` runs (if defined)
4. If severity is CRITICAL, the entire robot disables

---

## Core Concepts

### Fault Severity: How Bad Is It?

Every fault has a severity level that determines how the framework responds:

| Severity | Meaning | Robot Effect |
|----------|---------|--------------|
| **WARNING** | Potential issue arised; no action taken | Robot keeps running normally |
| **ERROR** | A module in your code has a problem; count it towards that component becoming unhealthy if applicable | Component may be disabled if faults persist |
| **CRITICAL** | Stop everything immediately | Robot disables this iteration |

Choose severity based on what the fault means for your robot:

```python
# Brownout warning (voltage sag)
self.raise_fault(self, FaultSeverity.WARNING, "Voltage dropped below 6V")

# Motor stalled for too long - subsystem is degraded
self.raise_fault(self, FaultSeverity.ERROR, "Shooter motor stalled")

# CAN bus dead - cannot control critical hardware
self.raise_fault(self, FaultSeverity.CRITICAL, "CAN bus offline")
```

### Component Health: Healthy vs Unhealthy

Each component has a **health state**:

- **HEALTHY** - Runs `execute()` every loop
- **UNHEALTHY (Faulted)** - Runs `on_faulted_periodic()` instead, skips `execute()`

A component becomes unhealthy after raising **10 consecutive ERROR or CRITICAL faults**. You can customize this threshold per-component:

```python
class Drivetrain(AdaptiveComponent):
    def __init__(self) -> None:
        super().__init__()
        self.fault_threshold = 5  # Stricter: unhealthy after 5 faults
    
    def execute(self) -> None:
        # Your code here
        pass
```

**Important:** WARNING faults do NOT count towards the threshold. Only ERROR and CRITICAL do.

### The Faulted Lifecycle

When your component becomes unhealthy:

1. `on_faulted_init()` is called once - go to a safe state
2. `on_faulted_periodic()` is called every loop - stay safe
3. `execute()` is NOT called while unhealthy

This gives you a chance to shut down gracefully:

```python
class Shooter(AdaptiveComponent):
    def __init__(self) -> None:
        super().__init__()
        self.speed = 0.0
    
    def execute(self) -> None:
        # Normal operation
        self.speed = self.requested_speed
    
    def on_faulted_init(self) -> None:
        # Component became unhealthy - stop immediately
        self.speed = 0.0
    
    def on_faulted_periodic(self) -> None:
        # Hold safe state
        self.speed = 0.0
```

Once unhealthy, a component stays that way for the rest of the match, or until the robot enters a disabled state (unless you manually set the health yourself).

### Locking: Disable a Component Manually

You can temporarily lock a component to disable it without faulting:

```python
# Something is wrong, but not a fault (e.g., user disabled it)
self.drivetrain.locked = True

# While locked:
# - execute() does NOT run
# - on_faulted_periodic() does NOT run
# - publish_telemetry() still runs (you need visibility!)
```

Locking is useful for user-initiated disables or when you want to bypass a subsystem without logging a fault.

### Fault Logging: Where Do Faults Go?

All faults are logged to a JSON file in the folder you configured:

```python
config = RobotConfig(
    fault_logging_folder="faults/",  # Faults saved here
)
```

Each match generates a new file like `FRC_20260414_011032_faults.json`. Later, review it and see what went wrong:

```json
[
  {
    "schedulable": "Drivetrain",
    "severity": "ERROR",
    "description": "Left motor encoder timeout",
    "timestamp": 123.456,
    "exception_type": "TimeoutError",
    "traceback": "..."
  },
  {
    "schedulable": null,
    "severity": "WARNING",
    "description": "Voltage brownout detected",
    "timestamp": 123.789,
    "exception_type": null,
    "traceback": null
  }
]
```

---

## API Reference

### Raising a Fault

Use `raise_fault()` (available on any `Schedulable` or `Faultable` object):

```python
def raise_fault(
    schedulable: Schedulable | None,
    severity: FaultSeverity,
    description: str,
    exception: Exception | None = None
) -> None
```

**Parameters:**

- `schedulable` - The component causing the fault (usually `self`). Can be `None` if it's a global issue.
- `severity` - One of `FaultSeverity.WARNING`, `FaultSeverity.ERROR`, or `FaultSeverity.CRITICAL`
- `description` - A clear, short message explaining what went wrong (50-100 chars recommended)
- `exception` - Optional Python exception for debugging. Include traceback in logs if provided.

**Examples:**

```python
# With no exception (detected programmatically)
self.raise_fault(
    self,
    FaultSeverity.ERROR,
    "Arm position out of bounds"
)

# With exception (caught from hardware layer)
try:
    encoder = self.io.get_encoder()
except TimeoutError as e:
    self.raise_fault(
        self,
        FaultSeverity.ERROR,
        "Encoder communication failed",
        exception=e
    )
```

### Component Health Properties

Check or modify component health:

```python
# Read-only: check current threshold
threshold = component.fault_threshold

# Modify threshold at runtime
component.fault_threshold = 5  # Stricter

# Check if locked
if component.locked:
    print("Component is locked")

# Lock or unlock
component.locked = True
component.locked = False
```

### Fault Severity Enum

```python
from adaptive_robot import FaultSeverity

FaultSeverity.WARNING    # Logged, robot runs normally
FaultSeverity.ERROR      # Logged, counts toward unhealthy
FaultSeverity.CRITICAL   # Logged, robot disables immediately
```

---

## Usage Examples

### Example 1: Hardware Motor Faults in IO Layer

Check motor faults when initializing hardware.

```python
from adaptive_robot.utils.talon_faults import TalonFaultLogger
from phoenix6.hardware import TalonFXS

class RealShooterIO:
    """IO layer for shooter hardware."""
    def __init__(self) -> None:
        self.left_motor = TalonFXS(1)
        self.right_motor = TalonFXS(2)
        
        # Check for hardware faults on startup
        self._check_motor_faults()
    
    def _check_motor_faults(self) -> None:
        """Checks for all motor faults and logs them."""
        fault_logger = TalonFaultLogger()
        
        for motor in (self.left_motor, self.right_motor):
            # Check both current and sticky faults
            fault_logger.report_talon_faults(motor, sticky=False, name=f"Shooter_{motor.device_id}")
            fault_logger.report_talon_faults(motor, sticky=True, name=f"Shooter_{motor.device_id}")
    
    def set_voltage(self, voltage: float) -> None:
        self.left_motor.setVoltage(voltage)
```

If any faults are found, `TalonFaultLogger` automatically raises them with `FaultSeverity.ERROR`.

### Example 2: Safe Defaults Pattern

Use the safe defaults pattern to quickly shut down when unhealthy.

```python
from adaptive_robot import AdaptiveComponent, BasicPriority, RequestArbitrator

class Drivetrain(AdaptiveComponent):
    def __init__(self, io) -> None:
        super().__init__()
        self.io = io
        self.linear_velocity_controller = RequestArbitrator()
        self.angular_velocity_controller = RequestArbitrator()
    
    def _safe_defaults(self) -> None:
        """Directly command safe values to the IO."""
        self.io.set_left_voltage(0.0)
        self.io.set_right_voltage(0.0)
    
    def on_enabled(self) -> None:
        self._safe_defaults()
    
    def on_disabled(self) -> None:
        self._safe_defaults()
    
    def on_faulted_init(self) -> None:
        # Component became unhealthy - stop immediately
        self._safe_defaults()
    
    def on_faulted_periodic(self) -> None:
        # Hold safe state every loop
        self._safe_defaults()
    
    def execute(self) -> None:
        # Normal operation
        linear = self.linear_velocity_controller.resolve().value
        angular = self.angular_velocity_controller.resolve().value
        
        self.io.set_left_voltage(linear)
        self.io.set_right_voltage(angular)
```

This pattern ensures that even if something goes wrong, the robot stops cleanly.

### Example 3: Request Arbitration with Safe State

Combine request arbitration with fault safety. From comp_tank_drive's intake:

```python
from adaptive_robot import AdaptiveComponent, BasicPriority, RequestArbitrator
from wpilib import RobotController

class Intake(AdaptiveComponent):
    def __init__(self, io) -> None:
        super().__init__()
        self.io = io
        self.percent_controller = RequestArbitrator()
    
    def request_percent(self, percent: float, priority: BasicPriority, source: str = "unknown") -> None:
        """Request speed from any source."""
        self.percent_controller.request(percent, priority.value, source)
    
    def _safe_defaults(self) -> None:
        self.io.set_voltage(0.0)
    
    def on_faulted_init(self) -> None:
        self._safe_defaults()
    
    def on_faulted_periodic(self) -> None:
        self._safe_defaults()
    
    def execute(self) -> None:
        # Resolve the highest priority request
        resolved_percent = self.percent_controller.resolve().value
        
        # Command hardware with battery voltage compensation
        voltage = resolved_percent * RobotController.getBatteryVoltage()
        self.io.set_voltage(voltage)
```

If a fault occurs and the component becomes unhealthy, `on_faulted_periodic()` holds it safely stopped.

### Example 4: Telemetry While Faulted

Publish telemetry even when unhealthy. This helps you debug what went wrong:

```python
from adaptive_robot import AdaptiveComponent, BasicPriority, RequestArbitrator

class IntakeArm(AdaptiveComponent):
    """From comp_tank_drive."""
    def __init__(self, io) -> None:
        super().__init__()
        self.io = io
        self.angle_controller = RequestArbitrator()
    
    def _safe_defaults(self) -> None:
        self.io.set_voltage(0.0)
    
    def on_faulted_init(self) -> None:
        self._safe_defaults()
    
    def on_faulted_periodic(self) -> None:
        self._safe_defaults()
    
    def publish_telemetry(self) -> None:
        """Runs EVERY loop, even when unhealthy. Critical for debugging."""
        self.publish_value("IntakeArm/sensorData/position", self.io.get_position())
        self.publish_value("IntakeArm/sensorData/velocity", self.io.get_velocity())
        self.publish_value("IntakeArm/sensorData/appliedVoltage", self.io.get_voltage())
        
        resolved = self.angle_controller.resolve()
        self.publish_value("IntakeArm/resolvedAngle/angle", resolved.value)
        self.publish_value("IntakeArm/resolvedAngle/source", resolved.source)
    
    def execute(self) -> None:
        """Only runs when healthy."""
        target_angle = self.angle_controller.resolve().value
        # ... control logic ...
```

Even if the component faults, you'll see the position, velocity, and voltage in NetworkTables. This is invaluable for match analysis.

---

## Common Patterns

**Use a fault when:** Something unexpected happens (sensor reads garbage, hardware disconnects, timeout).

**Use normal code when:** Expected conditions (shooter not at speed, intake empty, arm at limit).

```python
# Use a fault - unexpected
if encoder_reading < -50:
    self.raise_fault(self, FaultSeverity.ERROR, "Encoder malfunction")

# Use normal code - expected
if position > MAX_POSITION:
    self.position = MAX_POSITION
```

### Too many faults

If your component raises many faults in a row, it becomes unhealthy. This is intentional. But if you're raising faults constantly, you may have a design problem:

**Bad (raises fault every loop):**
```python
def execute(self):
    if self.motor.get_current() > THRESHOLD:
        self.raise_fault(...)  # Every loop while at high current
```

**Better (warn once per state change):**
```python
def execute(self):
    current = self.motor.get_current()
    if current > THRESHOLD and not self.over_current_logged:
        self.raise_fault(...)
        self.over_current_logged = True
    elif current <= THRESHOLD:
        self.over_current_logged = False
```
