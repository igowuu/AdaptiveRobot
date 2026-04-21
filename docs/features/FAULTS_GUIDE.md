# Faults & Fault Management

## Overview

The fault system provides a way to handle errors, warnings, and critical issues throughout your robot code. When a component or action detects a problem, it raises a fault with an appropriate severity level. The internal`FaultManager` catches these faults, logs them to JSON files, and decides whether the robot should be disabled.

This design keeps the main robot loop clean so that aults don't crash your code or propagate to unintended locations. Instead, they're captured, logged, and handled in all subclasses of AdaptiveComponent and any class inheriting from the Faultable module.

## Quick Start

Raise a fault from any component or action that inherits from `Faultable`:

```python
from adaptive_robot import AdaptiveComponent, Faultable, FaultSeverity

class MyComponent(AdaptiveComponent):
    def execute(self) -> None:
        try:
            result = self.sensor.read()
        except SensorError as e:
            self.raise_fault(
                component=self,
                severity=FaultSeverity.ERROR,
                description="Sensor read failed",
                exception=e
            )

    def check_limits(self) -> None:
        if self.position > MAX_POSITION:
            self.raise_fault(
                component=self,
                severity=FaultSeverity.WARNING,
                description=f"Position limit exceeded: {self.position}"
            )

    def detect_critical_issue(self) -> None:
        self.raise_fault(
            component=self,
            severity=FaultSeverity.CRITICAL,
            description="Motor overheating detected"
        )
```

---

## Fault Severity Levels

### FaultSeverity Enum

Faults are categorized by severity, which determines how they're handled:

```python
class FaultSeverity(StrEnum):
    WARNING = "WARNING"    # Logged, doesn't affect component or robot
    ERROR = "ERROR"        # Logged, contributes to component failures if applicable.
    CRITICAL = "CRITICAL"  # Logged, causes robot to disable
```

#### WARNING

- **Impact**: Informational only
- **Logged**: To JSON file
- **Robot Action**: None
- **Component Action**: Continues operating
- **Use Case**: Non-urgent issues like sensor calibration drifts, suboptimal performance

**Example:**
```python
self.raise_fault(
    component=self,
    severity=FaultSeverity.WARNING,
    description="Encoder drift detected."
)
```

#### ERROR

- **Impact**: Indicates a problem that may affect component health
- **Logged**: To JSON file
- **Robot Action**: None
- **Component Action**: Component can be marked as unhealthy
- **Use Case**: Recoverable errors, sensor timeouts, validation failures

**Example:**
```python
self.raise_fault(
    component=self,
    severity=FaultSeverity.ERROR,
    description="Motor temperature exceeded 70°C"
)
```

#### CRITICAL

- **Impact**: Stops the robot immediately
- **Logged**: To JSON file
- **Robot Action**: Robot disables (code raises)
- **Component Action**: Robot enters disabled mode
- **Use Case**: Safety-critical failures, unrecoverable errors, emergency stops

**Example:**
```python
self.raise_fault(
    component=self,
    severity=FaultSeverity.CRITICAL,
    description="Motor detected voltage spike, shutting down for safety"
)
```

---

## Fault Data Structure

### Fault Class

```python
@dataclass
class Fault:
    component: str | None          # Name of the component that raised the fault
    severity: FaultSeverity        # WARNING, ERROR, or CRITICAL
    description: str               # Human-readable fault description
    timestamp: float               # FPGA timestamp when fault occurred
    exception_type: str | None     # Name of the exception (if any)
    traceback: str | None          # Full traceback (if exception provided)
```

**Common Usage:**
```python
# In a fault handler (internal framework use)
fault = fault_exception.fault
print(f"Component: {fault.component}")
print(f"Severity: {fault.severity}")
print(f"Description: {fault.description}")
print(f"Exception: {fault.exception_type}")
```

---

## Raising Faults

### Faultable Interface

The `Faultable` base class provides the `raise_fault()` method for creating and raising faults. All `AdaptiveComponent` and `AsyncAction` classes inherit from `Faultable` by default.

#### `raise_fault(component: AdaptiveComponent | None, severity: FaultSeverity, description: str, exception: Exception | None = None) -> None`

Creates and raises a `FaultException`.

**Parameters:**
- `component`: The component raising the fault (usually `self`), or `None` for global faults
- `severity`: The fault severity level (`FaultSeverity.WARNING`, `.ERROR`, or `.CRITICAL`)
- `description`: A readable description of the fault
- `exception`: Optional exception object to capture for logging (automatically extracts traceback)

**Raises:**
- `FaultException`: Always raised with the created `Fault` object

**Example:**
```python
from adaptive_robot.faults.faults import FaultSeverity

class Motor(AdaptiveComponent):
    def periodic(self) -> None:
        try:
            voltage = self.motor_controller.get_output_voltage()
            if voltage > MAX_VOLTAGE:
                self.raise_fault(
                    component=self,
                    severity=FaultSeverity.ERROR,
                    description=f"Overvoltage: {voltage}V exceeds {MAX_VOLTAGE}V"
                )
        except Exception as e:
            self.raise_fault(
                component=self,
                severity=FaultSeverity.CRITICAL,
                description="Failed to read motor controller",
                exception=e
            )
```

---

## Fault Handling & Logging

### FaultManager

The `FaultManager` is the central hub for fault handling. It:
1. **Catches** faults raised by components and actions
2. **Reports** them to the console via WPILib
3. **Logs** them to JSON files
4. **Disables** the robot if a CRITICAL fault occurs

**Important**: Faults are handled at the `FaultManager` level and do not propagate to `AdaptiveRobot`. This means your robot won't crash due to unhandled faults.

#### Flow of Faults

```
Component/Action/Faultable raises fault ->
FaultException caught by FaultManager ->
Fault logged to console + JSON ->
If ERROR and consecutive error threshold met (default 10): Component disabled (where the error was raised) ->
If CRITICAL: Robot disables ->
Execution continues normally ->
```

**Example console output:**
```
[CRITICAL] Fault raised: Motor overheating detected
[ERROR] Fault raised: Encoder timeout
[WARNING] Fault raised: Sensor calibration drift detected
```

### FaultLogger

Faults are logged to JSON files in the `faults/` directory (configurable in `AdaptiveRobot.__init__()`). Each robot session gets its own timestamped file.

**File format:** `FRC_YYYYMMDD_HHMMSS_faults.json`

**Example JSON log:**
```json
[
  {
    "component": "Drivetrain",
    "severity": "ERROR",
    "description": "Motor current exceeded 100A",
    "timestamp": 1234567,
    "exception_type": null,
    "traceback": null
  },
  {
    "component": "Intake",
    "severity": "WARNING",
    "description": "Sensor read failed",
    "timestamp": 1234568,
    "exception_type": "TimeoutError",
    "traceback": "Traceback (most recent call last):\n  ..."
  }
]
```

---

## Common Patterns

### Hardware Limit Checking

```python
class Arm(AdaptiveComponent):
    MAX_POSITION = 100
    MIN_POSITION = 0
    
    def execute(self) -> None:
        position = self.encoder.getPosition()
        
        if position > self.MAX_POSITION:
            self.raise_fault(
                component=self,
                severity=FaultSeverity.WARNING,
                description=f"Position {position} exceeds max {self.MAX_POSITION}"
            )
        elif position < self.MIN_POSITION:
            self.raise_fault(
                component=self,
                severity=FaultSeverity.WARNING,
                description=f"Position {position} below min {self.MIN_POSITION}"
            )
```

### Autonomous Action with Fault Recovery

```python
def routine(self) -> AsyncAction:
    try:
        yield from self.move_to_position(target=50)
    except MotionError as e:
        self.raise_fault(
            component=None,
            severity=FaultSeverity.ERROR,
            description="Autonomous movement failed, aborting routine",
            exception=e
        )
```

---

## Integration with AdaptiveRobot

When a CRITICAL fault is raised, the robot disables itself:

1. **FaultManager catches the fault**
2. **Robot disables automatically**
3. **Disabled handlers execute**
4. **Robot returns to disabled state**
5. **Fault is logged**

---

## Best Practices

1. **Use explicit messages**: Include relevant values and context
   ```python
   # Good
   self.raise_fault(self, FaultSeverity.ERROR, f"Motor current {current}A exceeds limit {limit}A")
   
   # Bad
   self.raise_fault(self, FaultSeverity.ERROR, "Motor overcurrent")
   ```

2. **Match severity to impact**: Don't over-report CRITICAL faults
   ```python
   # WARNING for non-urgent issues
   self.raise_fault(self, FaultSeverity.WARNING, "Encoder needs recalibration")
   
   # ERROR for recoverable problems
   self.raise_fault(self, FaultSeverity.ERROR, "Sensor timeout, using cached value")
   
   # CRITICAL only for safety issues
   self.raise_fault(self, FaultSeverity.CRITICAL, "Voltage spike detected")
   ```

3. **Review fault logs after matches**: Use the JSON logs to diagnose issues
   ```
   faults/FRC_20260417_154230_faults.json
   ```

## Normal Exceptions

Normal Exception objects can be raised normally, although they will always crash the robot. They will not be logged, but the traceback will be printed in terminal.

## Additional Notes

FaultExceptions do not need to be explicitly caught by the coder, unless interception is needed (although it is heavily discouraged). 

---
