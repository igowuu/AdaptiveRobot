# Interfaces

## Overview

AdaptiveRobot is built on four core interfaces that allow flexible robot architecture without forcing a single inheritance model. Instead of everything inheriting from one base class, components can implement the specific capabilities they need.

**Core Interfaces:**
- `Schedulable` - Lifecycle hooks and health tracking for iterative execution
- `TelemetryPublishable` - Publish sensor data to NetworkTables each iteration
- `TunablePublishable` - Expose tunable parameters adjustable via NetworkTables
- `Faultable` - Report faults to the fault system

`AdaptiveComponent` is a convenience class that inherits all four interfaces. You can either extend `AdaptiveComponent` or implement the specific interfaces your class needs.

## Quick Start

Create a component by inheriting from `AdaptiveComponent` and implementing `execute()` (you must implement `execute()` since it's an abstract method):

```python
from adaptive_robot import AdaptiveComponent

class Drivetrain(AdaptiveComponent):
    def __init__(self) -> None:
        super().__init__()
        self.left_motor = PWMSparkMax(0)
        self.right_motor = PWMSparkMax(1)
    
    def execute(self) -> None:
        # Hardware commands go here
        self.left_motor.set(self.left_speed)
        self.right_motor.set(self.right_speed)
    
    def publish_telemetry(self) -> None:
        # Telemetry goes here
        self.publish_value("Drivetrain/Left Speed", self.left_speed)
        self.publish_value("Drivetrain/Right Speed", self.right_speed)

# Register it automatically in AdaptiveRobot
class MyRobot(AdaptiveRobot):
    def onRobotInit(self) -> None:
        self.drivetrain = Drivetrain()  # Auto-discovered into scheduler
```

Or, implement only the interfaces you need:

```python
from adaptive_robot import Schedulable, TelemetryPublishable

class MyCustomSubsystem(Schedulable, TelemetryPublishable):
    def execute(self) -> None:
        pass
    
    def publish_telemetry(self) -> None:
        pass
```

---

## Schedulable Interface

### Lifecycle

`Schedulable` objects follow a lifecycle that runs every robot iteration (50Hz by default):

#### `on_enabled() -> None`

Called once when the robot transitions from disabled to enabled.

- **When**: Exactly once per enable transition
- **Purpose**: Initialize state for the enabled mode
- **Use**: Reset PID controllers, zero encoders, enable motor safety

**Example:**
```python
def on_enabled(self) -> None:
    self.pid_controller.reset()
    self.encoder.reset()
    self.motor_safety.enable()
```

#### `on_disabled() -> None`

Called once when the robot transitions from enabled to disabled.

- **When**: Exactly once per disable transition
- **Purpose**: Safe shutdown for enabled mode
- **Use**: Stop motors, save state, disable hardware

**Example:**
```python
def on_disabled(self) -> None:
    self.left_motor.set(0)
    self.right_motor.set(0)
    self.save_telemetry_state()
```

#### `publish_telemetry() -> None`

Called every iteration before `execute()`.

- **When**: Every robot loop (50Hz by default)
- **Purpose**: Publish sensor readings and state to NetworkTables
- **Use**: Log motor speeds, positions, currents, temperatures, resolved Requests

**Example:**
```python
def publish_telemetry(self) -> None:
    self.publish_value("Arm/Position", self.encoder.getPosition())
    self.publish_value("Arm/Current", self.motor.getOutputCurrent())
    self.publish_value("Arm/Temperature", self.motor.getMotorTemperature())
```

#### `execute() -> None` (abstract)

Called every iteration after `publish_telemetry()`.

- **When**: Every robot loop (50Hz by default)
- **Purpose**: Command hardware based on requests and state
- **Use**: Set motor output, activate solenoids, run control loops

**Example:**
```python
def execute(self) -> None:
    # Get the active request from the request arbitrator
    speed_request = self.speed_controller.resolve()
    
    # Command hardware
    self.motor.set(speed_request.value)
```

#### `on_faulted_init() -> None`

Called once when the component is first marked as unhealthy (faulted).

- **When**: First time fault threshold is reached
- **Purpose**: Enter safe state and disable hardware
- **Use**: Retract mechanisms, disable motors, activate safety brakes

**Example:**
```python
def on_faulted_init(self) -> None:
    # Component is unhealthy
    self.motor.set(0)
    self.brake.engage()
```

#### `on_faulted_periodic() -> None`

Called every iteration while the component is unhealthy.

- **When**: Every iteration, only if component is faulted
- **Purpose**: Maintain safe state during fault
- **Use**: Hold safe position, log diagnostics

**Example:**
```python
def on_faulted_periodic(self) -> None:
    # Keep hardware in safe state
    self.motor.set(0)
```

---

## Component Context & Execution Flow

### Execution Order Each Iteration

```
1. Update Tunable Values (TunablePublishable)
2. Publish Telemetry (TelemetryPublishable)
3. Call Activation Methods (Schedulable: on_enabled/on_disabled)
4. Execute Schedulables (Schedulable: execute or on_faulted_periodic)
5. Run Actions (AsyncAction)
```

### Interface Contexts

Each interface uses a context object to manage shared state:

**TelemetryContext** - Injected into each `TelemetryPublishable`
```python
@dataclass
class TelemetryContext:
    telemetry: TelemetryPublisher           # Publish simple cached values
    struct_telemetry: TelemetryStructPublisher  # Publish WPIStruct objects
```

**TunableContext** - Injected into each `TunablePublishable`
```python
@dataclass
class TunableContext:
    tunables: list[TunableValue]           # Tunable parameters
    tunable_pids: list[TunablePIDController]  # Tunable PID controllers
```

These are automatically injected and managed by the framework.

---

## Health Management

### Component Health Status

Each component has a health state that determines which methods are called:

- **HEALTHY**: `on_enabled()` → `execute()` → `publish_telemetry()`
- **UNHEALTHY (FAULTED)**: `on_faulted_init()` → `on_faulted_periodic()`

#### `is_healthy() -> bool`

Returns True if the component is healthy.

**Example:**
```python
if self.drivetrain.is_healthy():
    print("Drivetrain is operational")
else:
    print("Drivetrain has faulted")
```

### Fault Threshold

The `ComponentScheduler` tracks consecutive faults. When a component raises `fault_threshold` consecutive faults, it's automatically marked unhealthy.

**Default threshold:** 10 consecutive faults

**Example scenario:**
```
Iteration 1: execute() raises FaultException → consecutive_faults = 1
Iteration 2: execute() raises FaultException → consecutive_faults = 2
...
Iteration 10: execute() raises FaultException → consecutive_faults = 10
             → Component marked UNHEALTHY
             → on_faulted_init() called
Iteration 11+: on_faulted_periodic() called instead of execute()
```

---

## Component Locking

### Locking and Unlocking Components

Lock a component to prevent its hooks from being called (except `publish_telemetry()`).

#### `locked` (property)

Get or set the locked state.

**Example:**
```python
# Lock a component
self.drivetrain.locked = True

if self.drivetrain.locked:
    print("Drivetrain is locked")

self.drivetrain.locked = False
```

**Use Case:**
```python
def onTeleopPeriodic(self) -> None:
    if self.emergency_stop_pressed:
        self.drivetrain.locked = True  # Prevent lifecycle calls
```

---

## Telemetry & Monitoring

### Publishing Values

#### `publish_value(directory: str, value: primitive_type) -> None`

Publishes a simple value (int, float, str, bool) to NetworkTables.

**Parameters:**
- `directory`: Path in NetworkTables (e.g., `"Drivetrain/leftSpeed"`)
- `value`: The value to publish (int, float, str, or bool)

**Example:**
```python
def publish_telemetry(self) -> None:
    self.publish_value("Arm/Position", self.encoder.getPosition())
    self.publish_value("Arm/Is Healthy", self.is_healthy())
    self.publish_value("Arm/Mode", "Automated")
```

#### `publish_struct_value(directory: str, value: object) -> None`

Publishes a WPIStruct object (Pose3d, Transform3d, etc) to NetworkTables.

**Parameters:**
- `directory`: Path in NetworkTables
- `value`: The WPIStruct object (e.g., Pose3d, Field3d)

**Performance Note:** WPIStruct publishing is heavier than simple values. Use sparingly.

**Example:**
```python
def publish_telemetry(self) -> None:
    self.publish_value("Robot/X", self.pose.X())
    # Or:
    self.publish_struct_value("Robot/Pose", self.pose)
```

---

## Tunable Parameters

Tunable parameters can be modified in real-time via NetworkTables without redeploying code.

### Tunable Values

#### `tunable(directory: str, default: TunableType) -> TunableValue`

Creates a tunable value that updates from NetworkTables each iteration.

**Parameters:**
- `directory`: NetworkTables path (e.g., `"Tunables/Drivetrain/Speed"`)
- `default`: Default value (int, float, str, or bool)

**Example:**
```python
class Drivetrain(AdaptiveComponent):
    def __init__(self) -> None:
        super().__init__()
        self.max_speed = self.tunable("Tunables/Drivetrain/MaxSpeed", 1.0)
        self.acceleration = self.tunable("Tunables/Drivetrain/Accel", 0.1)
    
    def execute(self) -> None:
        # Values auto-update from NetworkTables each iteration
        speed = self.max_speed.value * self.joystick.getRightY()
        self.motor.set(speed)
```

### Tunable PID Controllers

#### `tunablePID(kp: float, ki: float, kd: float, directory: str = "Tunables/PIDController", period: float = 0.02) -> TunablePIDController`

Creates a PID controller with tunable gains that update from NetworkTables.

**Parameters:**
- `kp`: Proportional gain (≥ 0)
- `ki`: Integral gain (≥ 0)
- `kd`: Derivative gain (≥ 0)
- `directory`: NetworkTables path for tunable gains
- `period`: Update period in seconds (default: 0.02 = 20ms)

**Example:**
```python
class Arm(AdaptiveComponent):
    def __init__(self) -> None:
        super().__init__()
        self.pid = self.tunablePID(
            kp=0.1,
            ki=0.0,
            kd=0.01,
            directory="Tunables/Arm/PID"
        )
        self.encoder = Encoder(0, 1)
    
    def execute(self) -> None:
        current_position = self.encoder.getDistance()
        output = self.pid.calculate(current_position, target=50)
        self.motor.set(output)
```

> As of the beta release on 4/18/26, tunables will NOT save after redeployment. This is guarenteed to be a feature once officially released.

---

## Raising Faults from Components

Components inherit from `Faultable` and can raise faults. See [FAULTS_GUIDE.md](./FAULTS_GUIDE.md) for details.

**Example:**
```python
from adaptive_robot.faults.faults import FaultSeverity

class Motor(AdaptiveComponent):
    def execute(self) -> None:
        voltage = self.motor_controller.getOutputVoltage()
        if voltage > 13.0:
            self.raise_fault(
                component=self,
                severity=FaultSeverity.ERROR,
                description=f"Motor overvoltage: {voltage}V"
            )
```

---

## Common Patterns

### Multi-Source Control with RequestArbitrator

```python
from adaptive_robot.requests import RequestArbitrator, BasicPriority

class Drivetrain(AdaptiveComponent):
    def __init__(self) -> None:
        super().__init__()
        self.left_speed = RequestArbitrator()
        self.right_speed = RequestArbitrator()
        self.left_motor = PWMSparkMax(0)
        self.right_motor = PWMSparkMax(1)
    
    def on_enabled(self) -> None:
        self.left_speed.clear()
        self.right_speed.clear()
    
    def execute(self) -> None:
        left_request = self.left_speed.resolve()
        right_request = self.right_speed.resolve()
        
        self.left_motor.set(left_request.value)
        self.right_motor.set(right_request.value)
    
    def publish_telemetry(self) -> None:
        left_request = self.left_speed.resolve()
        self.publish_value("Drivetrain/Left Speed", left_request.value)
        self.publish_value("Drivetrain/Left Source", left_request.source)
```

### Sensor Monitoring with Health Tracking

```python
class Arm(AdaptiveComponent):
    def __init__(self) -> None:
        super().__init__()
        self.encoder = Encoder(0, 1)
        self.consecutive_read_fails = 0
    
    def execute(self) -> None:
        try:
            position = self.encoder.getDistance()
            self.consecutive_read_fails = 0
        except TimeoutError as e:
            self.consecutive_read_fails += 1
            if self.consecutive_read_fails > 3:
                self.raise_fault(
                    component=self,
                    severity=FaultSeverity.ERROR,
                    description=f"Encoder timeout ({self.consecutive_read_fails} attempts)",
                    exception=e
                )
    
    def on_faulted_periodic(self) -> None:
        self.motor.set(0)
```

### PID Control Loop

```python
class Elevator(AdaptiveComponent):
    def __init__(self) -> None:
        super().__init__()
        self.motor = PWMSparkMax(0)
        self.encoder = Encoder(0, 1)

        self.pid = self.tunablePID(
            kp=0.5,
            ki=0.0,
            kd=0.1,
            directory="Tunables/Elevator/PID"
        )

        self.target_height = self.tunable("Tunables/Elevator/Target", 0.0)
    
    def execute(self) -> None:
        current_height = self.encoder.getDistance()
        target = self.target_height.value
        
        output = self.pid.calculate(current_height, target)
        self.motor.set(output)
    
    def publish_telemetry(self) -> None:
        self.publish_value("Elevator/Position", self.encoder.getDistance())
        self.publish_value("Elevator/Target", self.target_height.value)
```

### Component Health Recovery

```python
class Intake(AdaptiveComponent):
    def __init__(self) -> None:
        super().__init__()
        self.motor = PWMSparkMax(0)
        self.beam_sensor = DigitalInput(0)
    
    def execute(self) -> None:
        self.motor.set(self.speed_request)
    
    def on_faulted_init(self) -> None:
        self.motor.set(0)
    
    def on_faulted_periodic(self) -> None:
        # Keep hardware safe
        self.motor.set(0)
```

---

## Timing & Constraints

### Execution Guarantees

- All components are executed at **50Hz by default** (configurable via `AdaptiveRobot(period=0.02)`)
- `publish_telemetry()` runs **before** `execute()` every iteration
- Components are executed **in registration order** (order they appear in `AdaptiveRobot`)

### Important Constraints

1. **Blocking Operations**: Avoid long-running or blocking code in component methods
   ```python
   # Bad - blocks scheduler
   def execute(self):
       time.sleep(1)  # Do not do this
   
   # Good - quick operations
   def execute(self):
       self.motor.set(self.request)
   ```

2. **NetworkTables Access**: Tunable and telemetry operations are thread-safe and fast (caching), but limited in the values they support (mostly primitive types).

---
