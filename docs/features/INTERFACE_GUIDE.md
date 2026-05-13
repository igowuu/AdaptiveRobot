# Interfaces & Components

## Overview

Instead of forcing everything to inherit from one base class, AdaptiveRobot uses interfaces. Your component can implement only what it needs - telemetry publishing, tunables, scheduling, fault handling - or combine them all.

This flexibility means you can write components that just run code each loop, or complex ones that publish telemetry, accept tunable parameters, and handle faults. Mix and match.

The easiest way is to inherit from `AdaptiveComponent`, which brings all four interfaces together. But if you want something lighter, just implement the interfaces you actually need.

## Quick Start

The simplest component inherits from `AdaptiveComponent` and implements `execute()`:

```python
from adaptive_robot import AdaptiveComponent

class Drivetrain(AdaptiveComponent):
    def __init__(self, io) -> None:
        super().__init__()
        self.io = io
        self.motor_left = PWMSparkMax(0)
        self.motor_right = PWMSparkMax(1)
    
    def execute(self) -> None:
        # Command hardware each iteration
        self.motor_left.set(0.5)
        self.motor_right.set(0.5)
```

That's it. The framework will discover it and run it 50 times per second.

Want to add telemetry? Add `publish_telemetry()`:

```python
class Drivetrain(AdaptiveComponent):
    def __init__(self, io) -> None:
        super().__init__()
        self.io = io
    
    def execute(self) -> None:
        self.motor_left.set(0.5)
    
    def publish_telemetry(self) -> None:
        self.publish_value("Drivetrain/LeftSpeed", self.io.get_left_speed())
```

Or if you only need scheduling without telemetry or tunables, implement just `Schedulable`:

```python
from adaptive_robot import Schedulable

class MyController(Schedulable):
    def execute(self) -> None:
        pass  # Your code here
```

---

## The Four Interfaces

### Schedulable: Run Code Each Loop

Use this if your component needs lifecycle hooks (`on_enabled()`, `execute()`, `on_faulted_periodic()`, etc).

**Methods:**
- `on_enabled()` - Called once when robot enables
- `on_disabled()` - Called once when robot disables
- `execute()` - Called every loop while healthy (required)
- `on_faulted_init()` - Called once when component becomes unhealthy
- `on_faulted_periodic()` - Called every loop while unhealthy
- `locked` (property) - Disable lifecycle calls (except telemetry)

**When to use:** Anything that needs to be scheduled and run each iteartion.

### TelemetryPublishable: Publish Sensor Data

Use this to send values to NetworkTables every loop (motor speeds, encoder positions, etc).

**Methods:**
- `publish_telemetry()` - Called every loop to publish data
- `publish_value(key, value)` - Publish a simple value (int, float, bool, str)
- `publish_struct_value(key, object)` - Publish a WPIStruct (Pose3d, etc)

**When to use:** Whenever you want to log/monitor something in real-time.

### TunablePublishable: Live-Tunable Parameters

Use this to create values that can be changed in NetworkTables without redeploying.

**Methods:**
- `tunable(key, default)` - Create a tunable value
- `tunablePID(kp, ki, kd, directory)` - Create a tunable PID controller

**When to use:** For constants that you tweak during competition (speeds, gains, thresholds).

### Faultable: Report Problems

Use this to raise faults when something goes wrong.

**Methods:**
- `raise_fault(schedulable, severity, description, exception)` - Raise a fault

**When to use:** When you detect hardware issues, timeouts, or anomalies.

### AdaptiveComponent: All Four

Combines all four interfaces. Inherit from this if you want the full feature set.

---

## Lifecycle & Execution Order

When the robot runs, here's what happens each iteration:

```
1. Update tunable values from NetworkTables
2. Publish telemetry (publish_telemetry() called on all components)
3. Check enable/disable transitions
4. If healthy: execute()
   If unhealthy: on_faulted_periodic()
5. Run any scheduled actions
```

**Important:** `publish_telemetry()` always runs, even if the component is unhealthy. This is intentional - you need to know what's broken via networktables if something breaks.

---

## Schedulable in Detail

### Lifecycle Hooks

| Method | Called | Use for |
|--------|--------|---------|
| `on_enabled()` | Once when robot enables | Set safe state, zero encoders, zero sensors |
| `on_disabled()` | Once when robot disables | Stop motors, set safe state, cleanup |
| `execute()` | Every loop while healthy | Command hardware, run control loops |
| `on_faulted_init()` | Once when first unhealthy | Go to safe state (stop motors, etc) |
| `on_faulted_periodic()` | Every loop while unhealthy | Hold safe state |

### Health & Faults

Each component is either HEALTHY or UNHEALTHY (faulted).

When a component raises faults repeatedly, it gets marked unhealthy:
- **HEALTHY:** Runs `execute()` normally
- **UNHEALTHY:** Skips `execute()`, runs `on_faulted_periodic()` instead

**Default fault threshold:** 10 consecutive faults. Change it per-component if needed:

```python
class MyComponent(AdaptiveComponent):
    def __init__(self):
        super().__init__()
        self.fault_threshold = 5  # More sensitive
    
    def execute(self):
        try:
            sensor_value = self.encoder.get()
        except TimeoutError as e:
            self.raise_fault(self, FaultSeverity.ERROR, "Encoder timeout", e)
```

### Locking

Lock a component to prevent its lifecycle methods from running (except telemetry):

```python
# Lock when something is wrong
self.drivetrain.locked = True

# Locked components skip on_enabled/execute/on_faulted_periodic
# But telemetry still publishes
```

---

## TelemetryPublishable in Detail

### Publishing Values

Publish any primitive type (int, float, bool, str):

```python
def publish_telemetry(self) -> None:
    self.publish_value("Motor/Speed", 0.5)              # float
    self.publish_value("Intake/IsFull", True)            # bool
    self.publish_value("Mode", "Climbing")               # str
```

Or publish WPIStruct objects (Pose3d, Transform3d, etc):

```python
def publish_telemetry(self) -> None:
    self.publish_struct_value("Robot/Pose", self.pose)   # Pose3d
```

**Note:** WPIStruct publishing is heavier than simple values. Use it for important structural data, not every value.

---

## TunablePublishable in Detail

### Creating Tunables

Create a tunable value in `__init__()` and use `.value` to read it:

```python
class Drivetrain(AdaptiveComponent):
    def __init__(self):
        super().__init__()
        self.max_speed = self.tunable("Tunables/Drive/MaxSpeed", 1.0)
    
    def execute(self):
        # Value auto-updates from NetworkTables each loop
        speed = self.joystick.getRawAxis(1) * self.max_speed.value
        self.motor.set(speed)
```

### Tunable PID Controllers

Create a tunable PID that updates gains from NetworkTables:

```python
class Arm(AdaptiveComponent):
    def __init__(self):
        super().__init__()
        self.pid = self.tunablePID(
            kp=1.0,
            ki=0.0,
            kd=0.1,
            directory="Tunables/Arm/PID"
        )
        self.target = self.tunable("Tunables/Arm/Target", 0.0)
    
    def execute(self):
        current = self.encoder.getDistance()
        target = self.target.value
        output = self.pid.calculate(current, target)
        self.motor.set(output)
```

Now you can change kp, ki, kd, and target from NetworkTables during testing without redeploying.

---

## Faultable in Detail

### Raising Faults

Raise a fault when something goes wrong:

```python
from adaptive_robot.faults.faults import FaultSeverity

def execute(self):
    voltage = self.motor.getVoltage()
    if voltage > 13.0:
        self.raise_fault(
            schedulable=self,
            severity=FaultSeverity.ERROR,
            description=f"Motor overvoltage: {voltage}V"
        )
    
    try:
        data = self.encoder.read()
    except TimeoutError as e:
        self.raise_fault(
            schedulable=self,
            severity=FaultSeverity.CRITICAL,
            description="Encoder communication lost",
            exception=e
        )
```

**Severity levels:**
- `WARNING` - Minor issue, keep running
- `ERROR` - Recoverable problem, component go to safe state if consecutively raised
- `CRITICAL` - Major failure, robot will disable itself

---

## API Reference

### Schedulable Methods

```python
# Check health
if component.is_healthy():
    print("Running normally")
else:
    print("Faulted")

# Lock/unlock
component.locked = True
if component.locked:
    print("Lifecycle methods skipped (except telemetry)")

# Adjust fault threshold
component.fault_threshold = 5  # More sensitive than default 10
```

### TelemetryPublishable Methods

```python
# Publish primitives
self.publish_value("Key/Path", value)

# Publish WPIStruct
self.publish_struct_value("Pose/Key", pose_object)
```

### TunablePublishable Methods

```python
# Create a tunable value
my_speed = self.tunable("Tunables/Speed", 1.0)
print(my_speed.value)  # Read current value

# Create a tunable PID
my_pid = self.tunablePID(kp=0.5, ki=0, kd=0.1, directory="Tunables/PID")
output = my_pid.calculate(current, target)
```

### Faultable Methods

```python
# Raise a fault
self.raise_fault(
    schedulable=self,
    severity=FaultSeverity.ERROR,
    description="What went wrong",
    exception=optional_exception
)
```

---

## Usage Examples

Patterns from `examples/comp_tank_drive`.

### Example 1: Simple Component with Telemetry & Tuning

The shooter is a great example of `AdaptiveComponent` with request arbitration:

```python
class Shooter(AdaptiveComponent):
    def __init__(self, io) -> None:
        super().__init__()
        self.io = io
        self.velocity_controller = RequestArbitrator()
    
    def request_velocity(self, velocity, priority, source):
        """Public method for requesting velocity."""
        self.velocity_controller.request(velocity, priority.value, source)
    
    def on_enabled(self) -> None:
        """Safe defaults when enabled."""
        self.io.set_voltage(0.0)
    
    def execute(self) -> None:
        """Resolve request and command motor."""
        resolved = self.velocity_controller.resolve().value
        angular_velocity = resolved / ShooterConstants.FLYWHEEL_RADIUS
        percent = angular_velocity / ShooterConstants.MAX_VELOCITY
        self.io.set_voltage(percent * RobotController.getBatteryVoltage())
    
    def publish_telemetry(self) -> None:
        """Publish state for debugging."""
        self.publish_value("Shooter/Voltage", self.io.get_voltage())
        resolved = self.velocity_controller.resolve()
        self.publish_value("Shooter/ResolvedVelocity", resolved.value)
        self.publish_value("Shooter/Source", resolved.source)
```

### Example 2: Component with Tunable PID

The intake arm uses a tunable PID with gravity compensation:

```python
class IntakeArm(AdaptiveComponent):
    def __init__(self, io) -> None:
        super().__init__()
        self.io = io
        
        # Tunable PID for angle control
        self.pid = self.tunablePID(
            kp=5.0,
            ki=0.0,
            kd=0.8,
            directory="Tunables/IntakeArmPID"
        )
        
        self.angle_controller = RequestArbitrator()
    
    def request_angle(self, angle, priority, source):
        self.angle_controller.request(angle, priority.value, source)
    
    def execute(self) -> None:
        target_angle = self.angle_controller.resolve().value
        current_angle = self.io.get_position()
        
        # PID output + gravity compensation
        output = self.pid.calculate(current_angle, target_angle)
        self.io.set_voltage(output)
    
    def publish_telemetry(self) -> None:
        self.publish_value("Arm/Position", self.io.get_position())
        self.publish_value("Arm/Target", self.angle_controller.resolve().value)
```

### Example 3: Safe Defaults Pattern

All competition components use `_safe_defaults()` for consistency:

```python
class Drivetrain(AdaptiveComponent):
    def _safe_defaults(self) -> None:
        """Always set safe values here."""
        self.io.set_left_voltage(0.0)
        self.io.set_right_voltage(0.0)
    
    def on_enabled(self) -> None:
        self._safe_defaults()
    
    def on_disabled(self) -> None:
        self._safe_defaults()
    
    def on_faulted_init(self) -> None:
        self._safe_defaults()
    
    def on_faulted_periodic(self) -> None:
        self._safe_defaults()
```

This pattern ensures that no matter what happens (enable, disable, fault), the robot always stops safely.

### Example 4: Minimal Schedulable (No Telemetry)

Sometimes you don't need all four interfaces. A simple controller just needs scheduling:

```python
from adaptive_robot import Schedulable

class IntakeController(Schedulable):
    def __init__(self, intake: Intake, controller: Joystick):
        self.intake = intake
        self.controller = controller
    
    def execute(self) -> None:
        if not RobotState.isTeleop():
            return
        
        if self.controller.getRawButton(JoystickButton.INTAKE_GRABBING):
            self.intake.request_percent(1.0, BasicPriority.TELEOP, "teleop")
        elif self.controller.getRawButton(JoystickButton.INTAKE_RELEASING):
            self.intake.request_percent(-1.0, BasicPriority.TELEOP, "teleop")
        else:
            self.intake.request_percent(0.0, BasicPriority.TELEOP, "teleop")
```

### Example 5: Fault Handling

Detect and report hardware issues:

```python
class Shooter(AdaptiveComponent):
    def execute(self) -> None:
        try:
            velocity = self.io.get_velocity()
        except CommunicationError as e:
            self.raise_fault(
                schedulable=self,
                severity=FaultSeverity.ERROR,
                description="Failed to read shooter velocity",
                exception=e
            )
            return
        
        # Check for overvoltage
        voltage = self.io.get_voltage()
        if voltage > 13.5:
            self.raise_fault(
                schedulable=self,
                severity=FaultSeverity.WARNING,
                description=f"Shooter overvoltage: {voltage}V"
            )
```

---

## Things to Remember

- **`execute()` must be implemented.** It's abstract on `AdaptiveComponent` and `Schedulable`. If you only need telemetry/tunables, don't inherit from those - use `TelemetryPublishable` or `TunablePublishable` instead.
- **`publish_telemetry()` always runs.** Even if `execute()` is skipped (faulted, locked, etc), telemetry publishes. This is intentional so you can diagnose problems.
- **Tunables persist.** Between session startups, your tunable values will persist in a file, and will be read on the next startup.
- **Lock skips lifecycle, not telemetry.** If you lock a component, `on_enabled/execute/on_faulted_periodic` don't run, but `publish_telemetry()` still does.
- **Execution order matters.** Components run in the order they're registered. If component B depends on component A's output, make sure A is registered first.
- **Safe defaults pattern is recommended.** Call `_safe_defaults()` from `on_enabled`, `on_disabled`, `on_faulted_init`, and `on_faulted_periodic`. It ensures consistency.
