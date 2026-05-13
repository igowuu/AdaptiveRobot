# Profiling & Performance Monitoring

## Overview

Timing issues destroy robot performance. If a method takes too long, it causes loop overruns, dropped packets, and unpredictable behavior.

AdaptiveRobot includes automatic profiling so you can see how long each method takes. The framework profiles common methods (`execute()`, `publish_telemetry()`, etc.) automatically, and you can add custom profiling with a decorator. Profiling data is written to disk periodically so you can analyze it after the match.

This means you don't have to guess what's slow - you'll have data showing which methods are eating up your loop time.

## Quick Start

Profiling works automatically. No setup needed. Just look at the profiling files after your robot runs:

```python
from adaptive_robot import AdaptiveRobot, RobotConfig

config = RobotConfig(
    profiling_enabled=True,              # Enable profiling (default)
    profile_logging_frequency=10,        # Log every 10 seconds (default)
    profile_logging_folder="profiling/"  # Where to save files (default)
)

class MyRobot(AdaptiveRobot):
    def __init__(self) -> None:
        super().__init__(config)
```

That's it. When the robot runs, profiling data is automatically collected and saved to `profiling/FRC_YYYYMMDD_HHMMSS_profiles.json`.

If you want to profile a custom method, add the `@profile_method` decorator:

```python
from adaptive_robot import profile_method

class MyComponent:
    @profile_method
    def expensive_calculation(self) -> None:
        # This method's execution time will be profiled
        pass
```

---

## Core Concepts

### What Gets Profiled?

By default, these methods are automatically profiled:

- `execute()` on all `Schedulable` components
- `publish_telemetry()` on all `TelemetryPublishable` components

Any method you decorate with `@profile_method` is also profiled.

### Profiling Statistics

For each profiled method, the framework tracks:

| Statistic | Meaning |
|-----------|---------|
| `call_count` | How many times the method ran |
| `mean_time_seconds` | Average execution time |
| `min_time_seconds` | Fastest single execution |
| `max_time_seconds` | Slowest single execution |
| `stddev_seconds` | Standard deviation (consistency) |

**Example output:**

```json
{
  "timestamp": "2026-05-12T12:34:56.789123",
  "methods": [
    {
      "method_name": "Drivetrain.execute",
      "method_id": "Drivetrain.execute",
      "call_count": 600,
      "mean_time_seconds": 0.000234,
      "min_time_seconds": 0.000198,
      "max_time_seconds": 0.000567,
      "stddev_seconds": 0.000089
    },
    {
      "method_name": "Shooter.publish_telemetry",
      "method_id": "Shooter.publish_telemetry",
      "call_count": 600,
      "mean_time_seconds": 0.000045,
      "min_time_seconds": 0.000038,
      "max_time_seconds": 0.000089,
      "stddev_seconds": 0.000012
    }
  ]
}
```

### Logging Frequency

Profiling data is logged to disk:

1. **Periodically** - Every N amount of seconds (configurable, default 10 seconds)
2. **On disable** - When the robot becomes disabled (to preserve final match snapshot)

This means you get periodic updates during the match, plus a final snapshot when the robot is disabled.

---

## Configuration

Customize profiling behavior via `RobotConfig`:

```python
from adaptive_robot import AdaptiveRobot, RobotConfig

config = RobotConfig(
    profiling_enabled=True,              # Enable/disable all profiling
    profile_logging_frequency=10,        # Log every N seconds
    profile_logging_folder="profiling/"  # Where to save profile files
)

class MyRobot(AdaptiveRobot):
    def __init__(self) -> None:
        super().__init__(config)
```

### Configuration Options

| Option | Type | Default | Use for |
|--------|------|---------|---------|
| `profiling_enabled` | `bool` | `True` | Turn profiling on/off |
| `profile_logging_frequency` | `seconds` | `10` | How often to write data to disk |
| `profile_logging_folder` | `str` | `"profiling/"` | Folder where profiles are saved |

**Example: Disable profiling on competition robot:**

```python
config = RobotConfig(
    profiling_enabled=False  # Don't waste resources profiling on competition
)
```

**Example: Log more frequently during tuning:**

```python
config = RobotConfig(
    profile_logging_frequency=2  # Log every 2 seconds instead of 10
)
```

---

## Custom Profiling

### The `@profile_method` Decorator

Add profiling to any custom method by decorating it:

```python
from adaptive_robot import profile_method

class MyComponent(AdaptiveComponent):
    @profile_method
    def custom_calculation(self) -> float:
        """This method's timing will be profiled."""
        result = 0
        for i in range(1000):
            result += i ** 2
        return result
    
    def execute(self) -> None:
        # execute() is already profiled automatically
        value = self.custom_calculation()  # Profiled too
```

The decorator works on:
- Instance methods
- Static methods
- Functions

### What the Decorator Does

1. Records the start time (using FPGA timestamp)
2. Calls your method
3. Records the end time
4. Sends the elapsed time to the profiling context

If profiling is disabled, the decorator has minimal overhead.

### Multiple Decorators

You can combine profiling with other decorators:

```python
from ABC import abstractmethod
from adaptive_robot import profile_method

class MyComponent:
    @profile_method
    @abstractmethod
    def expensive_lookup(self, key: str) -> str:
        ...
```

---

### Looking for Loop Overruns

**Problem:** Driver station reports loop overruns

**Cause:** A method is taking longer than the loop period (typically 20ms for 50Hz)

**Solution:**

1. Check profiling data for methods with high `max_time_seconds`
2. If `max_time_seconds` > 0.015 (15ms), investigate that method
3. Example: If `Shooter.execute()` max is 18ms, it can cause overruns
4. Optimize that method or move work to a background thread

```python
# Look for this in profiling data:
# "max_time_seconds": 0.018  <- This exceeds 20ms loop period!
```

### Inconsistent Performance

**Problem:** `stddev_seconds` is large, meaning timing is inconsistent

**Cause:** 
- Method sometimes does more work (conditional branches)
- Garbage collection pauses
- Sensor communication timeouts

**Solution:**
- Use `max_time_seconds` for worst-case planning
- Add timeouts to sensor reads to prevent hangs
- Avoid allocating memory in fast loops (causes GC)

### Profiling Overhead

**Problem:** Is profiling itself slowing down the robot?

**Answer:** Minimal. Each profiled method adds ~0.0001ms overhead (one FPGA timestamp call). For 50 methods, that's ~0.005ms total per loop. Negligible.

**Note:** Logging to disk (every 10 seconds) takes a few milliseconds, but it happens in the background and doesn't block.

## Best Practices

### 1. Check Profiling Data After Every Match

Make it a habit to review profiling files. You'll catch performance regressions early.

### 2. Profile at Consistent Points

Compare profiling data from:
- Early match (fresh components)
- Mid match (thermal equilibrium)
- Late match (motor heating, battery sag)

This shows how the robot degrades over time.

### 3. Investigate Spikes in `max_time_seconds`

If a method usually takes 0.0001s but occasionally takes 0.005s, something is wrong:

```python
# Bad - huge variance
"mean_time_seconds": 0.0001,
"max_time_seconds": 0.05,  # 500x slower than average!
"stddev_seconds": 0.0008

# Good - consistent
"mean_time_seconds": 0.0001,
"max_time_seconds": 0.00044,  # Only slightly slower than average
"stddev_seconds": 0.00001
```

### 4. Profile Custom Methods Only When Needed

`@profile_method` is cheap, but don't decorate every method. Profile the suspect ones:

```python
# Do this - profile the expensive calculation
@profile_method
def path_planning_algorithm(self) -> list[Pose2d]:
    pass

# Don't do this - profiling trivial getters adds noise
@profile_method
def get_current_angle(self) -> float:
    return self.encoder.getPosition()
```

### 5. Use Profiling During Development

Profiling is most useful during development and tuning. On competition day, you can disable it if you're tight on CPU:

```python
IS_COMPETITION = True

config = RobotConfig(
    profiling_enabled=not IS_COMPETITION
)
```

---

## Troubleshooting

### "Profiling buffer has exceeded 100 pending snapshots"

**Cause:** Profiling data is being collected faster than it can be written to disk

**Solution:**
- Reduce `profile_logging_frequency` (log less often)
- Check disk speed (SD card vs internal storage)
- Disable profiling if disk I/O is the bottleneck

### Profiling Different on Real Robot vs Simulation

**Cause:** Real hardware has different performance characteristics

**Solution:** Profile on both. Simulation numbers are useful for relative comparison, but real robot is what matters.

---

## See Also

- [Robot Lifecycle & Configuration](./ROBOT_GUIDE.md) - How to set up RobotConfig
- [Interfaces & Components](./INTERFACE_GUIDE.md) - What methods are automatically profiled
- [Fault Handling](./FAULTS_GUIDE.md) - How faults interact with profiling
