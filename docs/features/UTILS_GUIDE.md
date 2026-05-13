# Utilities

## Overview

Common utility functions are scattered across every robotics codebase. Unit conversions, math helpers, hardware detection. Instead of reimplementing these each time, AdaptiveRobot includes a utilities module with frequently-needed functions.

These utilities cover:

- **Math** — Unit conversions between meters, rotations, radians, velocity formats
- **Hardware** — Detecting and reporting SparkMax and Talon faults
- **Data** — JSON persistence for telemetry and configuration
- **Action Selection** — Dynamic autonomous selection (covered separately in ACTION_CHOOSER_GUIDE.md)

Use these to reduce boilerplate and ensure consistency across your robot code.

---

## Math Utilities

### Basic Math

#### `clamp(value: float, lower: float, upper: float) -> float`

Constrains a value between min and max.

```python
from adaptive_robot.utils.math_utils import clamp

speed = clamp(joystick_input, -1.0, 1.0)  # Joystick output always -1 to 1
voltage = clamp(desired_voltage, -12.0, 12.0)  # Motor voltage limited
```

### Angle Conversion

#### `radians_to_degrees(radians) -> degrees`
#### `degrees_to_radians(degrees) -> radians`

Convert between angle units.

```python
from adaptive_robot.utils.math_utils import radians_to_degrees, degrees_to_radians

# From encoder (radians)
angle_rad = arm_encoder.getPosition()  # ~3.14 rad for 180°
angle_deg = radians_to_degrees(angle_rad)  # 180°

# To WPILib (wants radians)
target_deg = 45
target_rad = degrees_to_radians(target_deg)
pid_controller.setSetpoint(target_rad)
```

### Linear Motion Conversion

#### `rotations_to_meters(motor_rotations, wheel_diameter, gear_ratio=1.0) -> meters`
#### `meters_to_rotations(distance, wheel_diameter, gear_ratio=1.0) -> float`

Convert between motor rotations and linear distance traveled.

```python
from adaptive_robot.utils.math_utils import rotations_to_meters, meters_to_rotations

WHEEL_DIAMETER = 0.1524  # 6 inches in meters
GEAR_RATIO = 6.0  # 6:1 reduction

# Encoder reads 100 motor rotations
motor_rot = encoder.getPosition()
distance = rotations_to_meters(motor_rot, WHEEL_DIAMETER, GEAR_RATIO)  # ~0.4 meters

# We want to drive 2 meters
target_distance = 2.0
target_rotations = meters_to_rotations(target_distance, WHEEL_DIAMETER, GEAR_RATIO)  # ~62.8 rotations
```

### Encoder Tick Conversion

#### `ticks_to_meters(ticks, ticks_per_rotation, wheel_diameter, gear_ratio=1.0) -> float`
#### `meters_to_ticks(meters, ticks_per_rotation, wheel_diameter, gear_ratio=1.0) -> float`

Convert between encoder ticks and linear distance.

```python
from adaptive_robot.utils.math_utils import ticks_to_meters, meters_to_ticks

TICKS_PER_ROTATION = 2048  # REV encoder
WHEEL_DIAMETER = 0.1524
GEAR_RATIO = 6.0

# Encoder reads 100000 ticks
ticks = encoder.getTicks()
distance = ticks_to_meters(ticks, TICKS_PER_ROTATION, WHEEL_DIAMETER, GEAR_RATIO)

# Convert back
target_ticks = meters_to_ticks(5.0, TICKS_PER_ROTATION, WHEEL_DIAMETER, GEAR_RATIO)
```

### Velocity Conversion

#### `mps_to_rps(velocity_mps, wheel_diameter, gear_ratio=1.0) -> rps`
#### `rps_to_mps(motor_rps, wheel_diameter, gear_ratio=1.0) -> mps`

Convert between meters/second and motor rotations/second.

```python
from adaptive_robot.utils.math_utils import mps_to_rps, rps_to_mps

WHEEL_DIAMETER = 0.1524
GEAR_RATIO = 6.0

# Encoder reports 10 rotations/second
motor_rps = encoder.getVelocity()
mps = rps_to_mps(motor_rps, WHEEL_DIAMETER, GEAR_RATIO)

# We want 2 m/s
target_mps = 2.0
target_rps = mps_to_rps(target_mps, WHEEL_DIAMETER, GEAR_RATIO)
```

#### `ticks_per_100ms_to_mps(ticks_per_100ms, ticks_per_rotation, wheel_diameter, gear_ratio=1.0) -> mps`
#### `mps_to_ticks_per_100ms(mps, ticks_per_rotation, wheel_diameter, gear_ratio=1.0) -> ticks_per_100ms`

Convert between Talon-style ticks/100ms and m/s (Talons report velocity in ticks per 100 milliseconds).

```python
from adaptive_robot.utils.math_utils import ticks_per_100ms_to_mps

TICKS_PER_ROTATION = 2048
WHEEL_DIAMETER = 0.1524

# Talon reports 2000 ticks per 100ms
talon_velocity = 2000
mps = ticks_per_100ms_to_mps(talon_velocity, TICKS_PER_ROTATION, WHEEL_DIAMETER)
```

### Angular Velocity Conversion

#### `rps_to_radps(motor_rps, gear_ratio=1.0) -> radians_per_second`
#### `radps_to_rps(motor_radps, gear_ratio=1.0) -> rps`

Convert between rotations/second and radians/second.

```python
from adaptive_robot.utils.math_utils import rps_to_radps

GEAR_RATIO = 8.0

# Motor spins at 5 rps
motor_rps = 5.0
radps = rps_to_radps(motor_rps, GEAR_RATIO)  # Convert to rad/s
```

### Rotation to Radian Conversion

#### `rotations_to_radians(motor_rotations, gear_ratio=1.0) -> radians`
#### `radians_to_rotations(motor_radians, gear_ratio=1.0) -> float`

Convert between motor rotations and radians.

```python
from adaptive_robot.utils.math_utils import rotations_to_radians

# Arm at 3.5 rotations from zero
motor_rot = 3.5
angle_rad = rotations_to_radians(motor_rot)  # ~22 radians
```

---

## Hardware Fault Detection

### SparkMax Faults

#### `SparkMaxFaultLogger.report_sparkmax_faults(motor, sticky, name)`

Check for SparkMax faults and raise them as framework faults.

```python
from adaptive_robot import Faultable
from adaptive_robot.utils.sparkmax_faults import SparkMaxFaultLogger

class Drivetrain(Faultable):
    def __init__(self):
        self.logger = SparkMaxFaultLogger()
        self.motor = SparkMax(0)
    
    def execute(self):
        # Check for faults each iteration
        self.logger.report_sparkmax_faults(
            motor=self.motor,
            sticky=False,  # Current faults (not persistent)
            name="Left Motor"
        )
```

**Common SparkMax Faults:**

- `FIRMWARE` — Firmware issue
- `CAN` — CAN bus communication error
- `SENSOR` — Encoder/sensor disconnected
- `TEMPERATURE` — Motor overheated
- `BROWNOUT` — Voltage sag

#### `SparkMaxFaultLogger.report_sparkmax_warnings(motor, sticky, name)`

Report SparkMax warnings (non-critical issues).

```python
self.logger.report_sparkmax_warnings(
    motor=self.motor,
    sticky=False,
    name="Right Motor"
)
```

### Talon Faults

#### `TalonFaultLogger.report_talon_faults(motor, sticky, name)`

Check for Talon FX / FX-S faults.

```python
from adaptive_robot.utils.talon_faults import TalonFaultLogger
from phoenix6.hardware import TalonFX

logger = TalonFaultLogger()
motor = TalonFX(0)

logger.report_talon_faults(
    motor=motor,
    sticky=False,
    name="Shooter Motor"
)
```

**Common Talon Faults:**

- `SUPPLY_CURRENT_LIMIT` — Supply current limited
- `UNDERVOLTAGE` — Voltage too low
- `DEVICE_TEMP` — Motor overheated
- `HARDWARE` — Hardware failure
- `FORWARD_SOFT_LIMIT` / `REVERSE_SOFT_LIMIT` — Limit switch hit

---

## JSON Data I/O

### Saving Data

#### `log_json_data(file_path, data, append=False, indent=2, ensure_ascii=False)`

Write JSON data to a file.

```python
from adaptive_robot.utils.json_io import log_json_data

# Save configuration
config_data = {
    "shooter_kp": 5.0,
    "shooter_ki": 0.1,
    "drivetrain_max_speed": 4.5
}
log_json_data("config.json", config_data)

# Append telemetry data (list)
telemetry = [
    {"timestamp": 123.45, "speed": 2.1, "angle": 45.0},
    {"timestamp": 123.46, "speed": 2.2, "angle": 46.0}
]
log_json_data("telemetry.json", telemetry)

# Append more telemetry later
more_telemetry = [{"timestamp": 123.47, "speed": 2.3, "angle": 47.0}]
log_json_data("telemetry.json", more_telemetry, append=True)
```

**Parameters:**
- `file_path` — Where to save (string or Path)
- `data` — Dictionary or list to write
- `append` — If `True`, append to existing list (for time-series data)
- `indent` — Pretty-print indentation (2 or 4 recommended)
- `ensure_ascii` — If `False`, preserve Unicode characters

### Loading Data

#### `get_json_data(file_path, default=None) -> dict | list`

Read JSON data from a file.

```python
from adaptive_robot.utils.json_io import get_json_data

# Load configuration
config = get_json_data("config.json", default={})
shooter_kp = config.get("shooter_kp", 5.0)

# Load telemetry (safe if file doesn't exist)
telemetry = get_json_data("telemetry.json", default=[])
for entry in telemetry:
    print(f"Speed: {entry['speed']}")

# Raises error if file missing and no default
data = get_json_data("must_exist.json")  # FileNotFoundError if missing
```

**Parameters:**
- `file_path` — Path to JSON file
- `default` — Value returned if file doesn't exist

**Raises:**
- `FileNotFoundError` — File missing and no default provided
- `ValueError` — Invalid JSON format

---

---

## Best Practices

### 1. Keep Constants Centralized

Define all unit conversion constants in one place:

```python
# Good - constants file
class DrivetrainConstants:
    WHEEL_DIAMETER = 0.1524
    GEAR_RATIO = 6.0
    TICKS_PER_ROTATION = 2048

# Use everywhere
from constants import DrivetrainConstants
distance = ticks_to_meters(
    ticks,
    DrivetrainConstants.TICKS_PER_ROTATION,
    DrivetrainConstants.WHEEL_DIAMETER,
    DrivetrainConstants.GEAR_RATIO
)
```

### 2. Always Include Gear Ratio

Forgetting gear ratio is a common bug. Always pass it:

```python
# Good - explicit gear ratio
mps = rps_to_mps(rps, wheel_diameter, gear_ratio=6.0)

# Risky - relies on default (1.0)
mps = rps_to_mps(rps, wheel_diameter)  # Assumes no reduction!
```

### 3. Check Your Units

The biggest source of bugs is unit mismatch. Always verify:

```python
# inches → meters
wheel_diameter = 6.0 * 0.0254  # 6 inches to meters

# RPM → RPS
velocity_rps = velocity_rpm / 60.0

# Ticks (encoder) → rotations
rotations = ticks / ticks_per_rotation
```

### 4. Persist Important Data

Save tuned constants and fault logs:

```python
# Save after tuning
final_constants = {"shooter_kp": 5.2}
log_json_data("tuned_constants.json", final_constants)

# Load at startup
constants = get_json_data("tuned_constants.json", default={})
```

---

## API Reference

### All Available Functions

**Angle Conversion:**
- `radians_to_degrees(radians) -> degrees`
- `degrees_to_radians(degrees) -> radians`

**Linear Motion:**
- `rotations_to_meters(motor_rotations, wheel_diameter, gear_ratio=1.0) -> meters`
- `meters_to_rotations(distance, wheel_diameter, gear_ratio=1.0) -> float`
- `ticks_to_meters(ticks, ticks_per_rotation, wheel_diameter, gear_ratio=1.0) -> float`
- `meters_to_ticks(meters, ticks_per_rotation, wheel_diameter, gear_ratio=1.0) -> float`

**Velocity:**
- `mps_to_rps(velocity_mps, wheel_diameter, gear_ratio=1.0) -> rps`
- `rps_to_mps(motor_rps, wheel_diameter, gear_ratio=1.0) -> mps`
- `ticks_per_100ms_to_mps(ticks_per_100ms, ticks_per_rotation, wheel_diameter, gear_ratio=1.0) -> mps`
- `mps_to_ticks_per_100ms(mps, ticks_per_rotation, wheel_diameter, gear_ratio=1.0) -> ticks_per_100ms`

**Angular Velocity:**
- `rps_to_radps(motor_rps, gear_ratio=1.0) -> radians_per_second`
- `radps_to_rps(motor_radps, gear_ratio=1.0) -> rps`
- `rotations_to_radians(motor_rotations, gear_ratio=1.0) -> radians`
- `radians_to_rotations(motor_radians, gear_ratio=1.0) -> float`

**Other:**
- `clamp(value, lower, upper) -> float`

---

## See Also

- [Fault Handling](./FAULTS_GUIDE.md) — How faults are reported
- [Interfaces & Components](./INTERFACE_GUIDE.md) — Where to use utilities in components
