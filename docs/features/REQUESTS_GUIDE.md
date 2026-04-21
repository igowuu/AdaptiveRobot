# AxisController & Requests

## Overview

The request-based arbitration system provides a clean way to manage competing commands to a single axis or subsystem. Multiple sources (teleop, autonomous, safety systems) can submit requests with different priorities, and the `AxisController` automatically selects the highest-priority active request each iteration.

This eliminates the need for complex conditional logic and makes it easy to add new command sources without breaking existing code.

## Quick Start

Create an AxisController for each axis (e.g. vx, vy, and omega would be separate) and submit requests from different sources:

```python
from adaptive_robot import AdaptiveComponent, AxisController, BasicPriority

class Intake(AdaptiveComponent):
    def __init__(self) -> None:
        super().__init__()

        self.percent_controller = AxisController()
        self.motor = ...

    def request_percent(
        self,
        percent: float,
        priority: BasicPriority, 
        source: str = "unknown"
    ) -> None:
        """
        Requests a velocity to the intake's percent controller.
        """
        self.percent_controller.request(percent, priority.value, source)

    def execute(self) -> None:
        # Resolves the request with a higher PRIORITY value. 
        # If PRIORITY values are equal, uses the most recent request as a tiebreaker for the winner.
        request = self.speed_controller.resolve()
        motor.set(request.value)
```

---

## AxisController

### Constructor

#### `__init__(default_value: float = 0.0, default_priority: int = -1, default_source: str = 'default') -> None`

Creates an AxisController to manage request-based arbitration for a single axis.

**Parameters:**
- `default_value`: The value returned when no active requests exist (default: `0.0`)
- `default_priority`: The priority assigned to the default request (must be different from all submitted request priorities; default: `-1`)
- `default_source`: The source name for the default request (default: `'default'`)

**Example:**
```python
# Drivetrain speed controller defaults to 0.0 at priority -1
speed_controller = AxisController(
    default_value=0.0,
    default_priority=-1,
    default_source="drivetrain_speed"
)
```

### Core Concepts

#### Request Arbitration
When multiple requests are active, the AxisController selects the winner based on:
1. **Highest priority** wins
2. **If tied**, the most recently submitted request wins (newer timestamp)
3. **If same iteration** (tied timestamps), the most recently inserted wins

#### Timeout Behavior
Each request has a timeout. If a request is not resubmitted before its timeout expires, it is automatically removed. This prevents stale requests from controlling the axis.

**Example:**
```python
# Teleop request times out after 0.2 seconds if not resubmitted
speed_controller.request(
    value=0.5,
    priority=BasicPriority.TELEOP.value,
    source="teleop",
    timeout=0.2
)

# So if no new request comes in within 0.2s, the controller reverts to default
```

---

## AxisRequest

The immutable data class that represents a single request.

```python
@dataclass(frozen=True)
class AxisRequest:
    value: float              # The commanded value
    priority: int             # Priority relative to other requests
    timeout: seconds = 0.2    # How long this request remains active
    source: str = "unknown"   # Name of the request source
    timestamp: seconds        # When the request was created (auto-set)
```

**Common Usage:**
```python
request = speed_controller.resolve()
print(f"Value: {request.value}")
print(f"Priority: {request.priority}")
print(f"Source: {request.source}")
```

---

## BasicPriority

A convenience enum for common priority levels. You can use these values or define your own priority integers.

```python
class BasicPriority(Enum):
    SAFETY = 3       # Highest priority - safety systems
    AUTO = 2         # Mid priority - autonomous routines
    TELEOP = 1       # Low priority - driver input
```

**Usage:**
```python
# Use the enum values
self.controller.request(
    value=safe_position,
    priority=BasicPriority.SAFETY.value,
    source="safety_limit"
)

# Or define custom priorities
self.controller.request(
    value=user_speed,
    priority=CustomPriority.CUSTOM.value,
    source="custom_source"
)
```

---

## AxisController API

### Submitting Requests

##### `request(value: float, priority: int, source: str = "unknown", timeout: seconds = 0.2) -> None`

Submits a request to the AxisController. If a request with the same source already exists, it is replaced.

**Parameters:**
- `value`: The commanded value for this request
- `priority`: Priority level relative to other requests. Must be different from the default priority
- `source`: Unique identifier for the request source (alphanumeric + underscores, max 50 chars)
- `timeout`: How long this request remains active (seconds). Range: `0 < timeout <= 10.0`

**Raises:**
- `ValueError`: If source name is invalid or timeout is out of bounds
- `RuntimeError`: If priority equals the default priority

**Example:**
```python
def onTeleopPeriodic(self) -> None:
    speed = self.joystick.getLeftY()
    self.speed_controller.request(
        value=speed,
        priority=BasicPriority.TELEOP.value,
        source="teleop_driver",
        timeout=0.2
    )
```

### Resolving Requests

##### `resolve() -> AxisRequest`

Returns the currently active request based on arbitration rules. Automatically removes timed-out requests.

**Arbitration order:**
1. Highest priority wins
2. If tied, most recent timestamp wins
3. If no valid requests, returns the default request

**Returns:** The winning `AxisRequest`

**Example:**
```python
def execute(self) -> None:
    active_request = self.speed_controller.resolve()
    motor.set(active_request.value)

    print(f"Active source: {active_request.source}")
```

### Clearing Requests

##### `clear() -> None`

Removes all pending requests. The next `resolve()` call will return the default request.

**Example:**
```python
def onDisabledInit(self) -> None:
    self.speed_controller.clear()
```

### Enabling & Disabling

##### `set_enabled(enabled: bool) -> None`

Enables or disables the AxisController. When disabled, `resolve()` always returns the default request and `request()` calls are ignored.

**Parameters:**
- `enabled`: True to enable, False to disable

**Example:**
```python
def on_safety_fault(self) -> None:
    # Disable the controller when safety fault occurs
    self.speed_controller.set_enabled(False)

def on_safety_recovered(self) -> None:
    # Re-enable after fault is cleared
    self.speed_controller.set_enabled(True)
```

##### `is_enabled() -> bool`

Returns the current enabled state.

**Example:**
```python
if not self.speed_controller.is_enabled():
    print("Speed controller is disabled")
```

### Introspection

##### `get_default_request() -> AxisRequest`

Returns the default request used when no valid requests are active.

**Example:**
```python
default = self.speed_controller.get_default_request()
print(f"Default value: {default.value}")
```

##### `get_pending_request_count() -> int`

Returns the number of currently active (non-timed-out) requests.

**Example:**
```python
if self.speed_controller.get_pending_request_count() > 0:
    print(f"Active requests: {self.speed_controller.get_pending_request_count()}")
```

##### `last_request` (property)

Returns the most recently resolved request, even if it's no longer active.

**Example:**
```python
prev = self.speed_controller.last_request
print(f"Last source: {prev.source}")
```

---

## Common Examples

### Multi Source Control

```python
class Drivetrain(AdaptiveComponent):
    def __init__(self) -> None:
        super().__init__()
        self.left_speed = AxisController()
        self.right_speed = AxisController()
    
    def onTeleopPeriodic(self) -> None:
        left = self.joystick.getLeftY()
        right = self.joystick.getRightY()
        
        self.left_speed.request(left, BasicPriority.TELEOP.value, "teleop")
        self.right_speed.request(right, BasicPriority.TELEOP.value, "teleop")
    
    def periodic(self) -> None:
        left_request = self.left_speed.resolve()
        right_request = self.right_speed.resolve()
        
        self.left_motor.set(left_request.value)
        self.right_motor.set(right_request.value)
```

### Autonomous with Safety Override

```python
def onAutonomousInit(self) -> None:
    self.speed_controller.request(
        value=0.5,
        priority=BasicPriority.AUTO.value,
        source="auto_drive"
    )

def onAutonomousPeriodic(self) -> None:
    # Safety system can override
    if self.too_close_to_wall():
        self.speed_controller.request(
            value=0.0,
            priority=BasicPriority.SAFETY.value,
            source="safety_wall_detector"
        )
```

### Request Timeout Strategy

```python
# Teleop requests timeout quickly (operator might release joystick)
self.speed_controller.request(speed, BasicPriority.TELEOP.value, "teleop", timeout=0.2)

# Autonomous requests timeout longer
self.speed_controller.request(speed, BasicPriority.AUTO.value, "auto_drive", timeout=1.0)

# Safety requests timeout slowly
self.speed_controller.request(safe_val, BasicPriority.SAFETY.value, "safety_limit", timeout=5.0)
```

---
