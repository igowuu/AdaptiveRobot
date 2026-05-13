# System Identification (SysID)

## Overview

Tuning feedforward and feedback gains is hard. You have to guess PID constants, test them, adjust, repeat. It's slow and imprecise.

System Identification (SysID) is a tool that automates this. You run your mechanism through controlled tests, collect data, and WPILib's SysID utility generates tuned feedforward constants automatically. Then you plug those into your control loop and it works.

AdaptiveRobot integrates with WPILib's SysID system. You wrap your mechanism, define test parameters, run the tests in test mode, and export the data to WPILib's SysID tool. The tool analyzes the data and gives you gains to use.

## Quick Start

Create a mechanism wrapper and SysID tests:

```python
from adaptive_robot import AdaptiveRobot
from adaptive_robot.sysid import SysIdTest, Mechanism, Config

class MyRobot(AdaptiveRobot):
    def onTestInit(self) -> None:
        # Define how to command and measure your mechanism
        mechanism = Mechanism(
            command_voltage=lambda v: self.shooter_motor.setVoltage(v),
            get_voltage=lambda: self.shooter_motor.getAppliedOutput() * 12.0,
            get_distance=lambda: self.shooter_encoder.getPosition(),
            get_velocity=lambda: self.shooter_encoder.getVelocity(),
            name="Shooter"
        )
        
        # Create all four SysID tests (quasistatic forward/reverse, dynamic forward/reverse)
        config = Config()
        qs_forward, qs_reverse, dyn_forward, dyn_reverse = SysIdTest.create_all_tests(
            mechanism=mechanism,
            config=config
        )
        
        # Store them for access in testPeriodic
        self.sysid_tests = [qs_forward, qs_reverse, dyn_forward, dyn_reverse]
        self.current_test_index = 0
```

Then schedule them from test mode:

```python
def onTestPeriodic(self) -> None:
    # Run the current test (example)
    self.schedule_action(sysid_arm_routine.run(), "sysid_arm")
```

---

## Core Concepts

### Four Test Types

SysID runs four standardized tests:

| Test | Purpose | Movement |
|------|---------|----------|
| **Quasistatic Forward** | Ramp voltage up slowly in forward direction | Smooth acceleration |
| **Quasistatic Reverse** | Ramp voltage up slowly in reverse direction | Smooth acceleration |
| **Dynamic Forward** | Apply step voltage in forward direction | Instant acceleration |
| **Dynamic Reverse** | Apply step voltage in reverse direction | Instant acceleration |

Each test shows how your mechanism responds to different inputs, letting the tool fit both static friction and dynamic response models.

### Data Collection

Each test runs for a configurable timeout (default 10 seconds) and collects:

- Applied voltage
- Motor position
- Motor velocity
- Timestamps

WPILib's `SysIdRoutineLog` handles data logging automatically. The data is stored as a standard WPILib log file that the SysID tool understands.

### Mechanism Wrapper

`Mechanism` abstracts how you control your hardware. You provide lambdas that:

1. **Command voltage** — Apply voltage to your motor
2. **Read applied voltage** — What voltage is actually being output
3. **Read position** — Linear or angular position (in meters or radians)
4. **Read velocity** — How fast it's moving

```python
mechanism = Mechanism(
    command_voltage=lambda v: motor.setVoltage(v),
    get_voltage=lambda: motor.getAppliedOutput() * battery_voltage,
    get_distance=lambda: encoder.getPosition(),
    get_velocity=lambda: encoder.getVelocity(),
    name="MyMechanism"
)
```

### Configuration

Customize test behavior via `Config`:

```python
from adaptive_robot.sysid import Config

config = Config(
    rampRate=0.5,              # Voltage ramp rate for quasistatic (V/s). Default: 1.0
    stepVoltage=7.0,           # Applied voltage for dynamic tests. Default: 7.0
    timeout=10.0,              # Test duration (seconds). Default: 10.0
    maximum_volts=12.0         # Cap on output voltage. Default: 12.0
)
```

**Common tuning:**

- **Slow motor?** Increase `stepVoltage` or decrease `rampRate`
- **Test ends too soon?** Increase `timeout`
- **Motor getting too hot?** Decrease `stepVoltage` or `timeout`

---

## Creating Tests

### Standard Approach

Create all four tests at once using the factory method:

```python
mechanism = Mechanism(...)
config = Config()

qs_forward, qs_reverse, dyn_forward, dyn_reverse = SysIdTest.create_all_tests(
    mechanism=mechanism,
    config=config
)
```

Returns a tuple of four ready-to-run `SysIdTest` objects.

### Custom Tests

If you need specific tests, create them individually:

```python
from adaptive_robot.sysid import SysIdTest, Direction, SysIdTestType, Mechanism, Config
from wpilib.sysid import SysIdRoutineLog

logger = SysIdRoutineLog("MyMechanism")

test = SysIdTest(
    direction=Direction.kForward,
    config=Config(rampRate=0.5),
    mechanism=mechanism,
    test_type=SysIdTestType.QUASISTATIC,
    logger=logger
)
```

---

## Running Tests

### As Async Actions

Schedule tests in test mode just like autonomous routines:

```python
def onTestInit(self) -> None:
    mechanism = Mechanism(...)
    qs_fwd, qs_rev, dyn_fwd, dyn_rev = SysIdTest.create_all_tests(mechanism, Config())
    self.tests = [qs_fwd, qs_rev, dyn_fwd, dyn_rev]

def onTestPeriodic(self) -> None:
    # Run first test that hasn't finished
    for test in self.tests:
        if test.is_running() or not self.has_run_yet(test):
            if not test.is_running():
                self.schedule_action(test.run(), "sysid")
            break
```

### Test Flow

```
1. Test starts (timer begins)
2. Each iteration:
   - Apply voltage (ramp or step)
   - Read mechanism state
   - Log voltage, position, velocity
3. Timeout reached or robot disabled
4. Stop mechanism (voltage → 0)
5. Test complete
```

### Checking Status

```python
test = qs_forward

# Is it running?
if test.is_running():
    print("Test in progress")

# Wait for completion
# (either by checking is_running() or letting it auto-stop)
```

---

## Exporting and Analyzing Data

### Data Location

SysID logs are saved to your robot's deploy directory and synced when you run WPILib's SysID tool.

Typically found in:
- `roboRIO:/media/sda1/` (on the robot)
- Synced to local machine when using WPILib SysID GUI

### Using WPILib SysID Tool

1. **Collect data** — Run all four tests (robot syncs logs to tool)
2. **Analyze** — Tool fits models and generates constants
3. **Get gains** — Tool outputs `kS`, `kV`, `kA` (feedforward) and `kP`, `kI`, `kD` (feedback)

### Common Outputs

Example from a shooter mechanism:

```
Ks (static friction): 0.12 volts
Kv (velocity): 0.11 volts·s/m
Ka (acceleration): 0.002 volts·s²/m
Kp (proportional): 5.0
Ki (integral): 0.0
Kd (derivative): 0.2
```

Use these in your control loops:

---

## Best Practices

### 1. Test in Isolated Mode

Disable other mechanisms during SysID testing to avoid interference (since SysID tests likely bypass the request arbitrator system):

```python
def onTestInit(self) -> None:
    # Disable normal subsystems
    self.drivetrain.locked = True
    self.arm.locked = True
    
    # Only shooter SysID runs
    mechanism = Mechanism(...)
    # ... rest of setup
```

### 2. Run Full Test Sequence

Always run all four tests (quasistatic forward/reverse, dynamic forward/reverse) for best results:

```python
# Good - complete test suite
qs_fwd, qs_rev, dyn_fwd, dyn_rev = SysIdTest.create_all_tests(mechanism, config)
tests = [qs_fwd, qs_rev, dyn_fwd, dyn_rev]

# Avoid - only one direction
tests = [qs_fwd, dyn_fwd]  # Incomplete data
```

### 3. Let Each Test Fully Complete

Don't interrupt tests early unless for safety. Let them run to timeout:

```python
# Good - run full duration
self.schedule_action(test.run(), "sysid")
```

### 4. Use Consistent Configuration

Use the same `Config` for all four tests so results are comparable:

```python
# Good - consistent config
config = Config(stepVoltage=7.0, timeout=10.0)
tests = SysIdTest.create_all_tests(mechanism, config)
```

### 5. Verify Encoder Readings

Make sure your position and velocity readings are correct:

```python
mechanism = Mechanism(
    command_voltage=lambda v: motor.setVoltage(v),
    get_voltage=lambda: motor.getAppliedOutput() * 12.0,
    get_distance=lambda: encoder.getPosition(),  # Check: in meters?
    get_velocity=lambda: encoder.getVelocity(),  # Check: in m/s?
    name="Shooter"
)
```

Incorrect encoder conversions lead to incorrect gains.

---

## Common Issues

### Test Doesn't Start

**Problem:** `is_running()` returns `False` immediately

**Cause:** Robot is disabled or test logic is wrong

**Solution:**
```python
def onTestPeriodic(self):
    # Make sure robot is enabled in test mode
    if not test.is_running():
        # Start it
        self.schedule_action(test.run(), "sysid")
```

### All Tests Show Same Data

**Problem:** Forward and reverse tests look identical

**Cause:** Mechanism isn't actually moving, or encoder isn't connected

**Solution:**
- Physically verify motor moves when voltage is applied
- Check encoder readings change during tests
- Verify voltage is actually being output

### Gains Are Huge or Negative

**Problem:** Calculated `kV` or `kA` is 100+ or negative

**Cause:** 
- Wrong units (position in counts instead of meters)
- Encoder backwards
- Motor backwards

**Solution:**
- Verify encoder units: position should be in meters (not rotations or ticks)
- Check velocity has correct sign (positive = forward)
- Verify motor moves in expected direction

### Test Stops Immediately

**Problem:** Test runs for only 0.1 seconds

**Cause:** Robot got disabled or timeout is too short

**Solution:**
```python
config = Config(timeout=10.0)  # Increase timeout
# And make sure robot stays enabled in test mode
```

---

## API Reference

### Mechanism

```python
mechanism = Mechanism(
    command_voltage: Callable[[float], None],
    get_voltage: Callable[[], float],
    get_distance: Callable[[], float],
    get_velocity: Callable[[], float],
    name: str
)
```

**Parameters:**
- `command_voltage` — Function that applies voltage (V) to motor
- `get_voltage` — Function returning applied voltage (V)
- `get_distance` — Function returning position (m or rad)
- `get_velocity` — Function returning velocity (m/s or rad/s)
- `name` — Mechanism name for logging

### Config

```python
config = Config(
    rampRate: float = 1.0,              # Voltage/second for quasistatic
    stepVoltage: float = 7.0,           # Voltage for dynamic tests
    timeout: float = 10.0,              # Test duration (seconds)
    maximum_volts: float = 12.0         # Voltage cap
)
```

### SysIdTest

```python
test = SysIdTest.create_all_tests(mechanism, config)
# Returns: (qs_forward, qs_reverse, dyn_forward, dyn_reverse)
```

**Methods:**
- `run()` → Generator (use with `schedule_action()`)
- `is_running()` → `bool` (True while test active)

---

## See Also

- [AsyncAction & Autonomous Routines](./ACTIONS_GUIDE.md) — How to schedule tests as actions
- [Robot Lifecycle & Configuration](./ROBOT_GUIDE.md) — Test mode hooks
- [WPILib SysID Documentation](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/index.html) — Detailed SysID guide
