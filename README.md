# AdaptiveRobot Framework

> Robot code that is easier to debug, safer under failure, and simpler to coordinate.

## Overview

AdaptiveRobot is a Python framework for FRC teams that handles the 
repetitive, safety-critical, and debugging-heavy parts of robot code 
so teams can focus on their robot code. 

Features:
- **Request-based arbitration** - Multiple systems (teleop, autonomous, 
  safety) safely submit commands without conflicts or blocking.
- **Observability** - Telemetry, tunables, profiling, and 
  structured fault logging are built-in to the framework.
- **Component Resilience** - Automatic health tracking and graceful 
  degradation when systems fail, while loudly logging all errors for review.
- **Pythonic autonomous** - Generator-based async actions read naturally and allow for python-native syntax.
- **Interface flexibility** - Components implement only what they need; 
  no mandatory lifecycle boilerplate.

AdaptiveRobot solves problems with observability, safety, and diagnostics while staying flexible about how you organize code.

- **What autonomous looks like:**
```python
    yield from robot.drivetrain.move_forward(distance=5)
    yield from wait(0.5)
    yield from robot.intake.spin_up()
    yield from parallel(
        robot.drivetrain.move_forward(distance=5.0),
        robot.intake.collect_pieces()
    )
```

**What requests look like:**
```python
    self.linear_velocity_controller = RequestArbitrator()
    self.linear_velocity_controller.request(
        value=1.0,
        priority=BasicPriority.TELEOP,
        source="teleop"
    )
    self.linear_velocity_controller.request(
        value=1.5,
        priority=BasicPriority.AUTO,
        source="auto"
    )

    # Higher-priority requests automatically override lower-priority requests, allowing safety and autonomous systems to preempt teleop behavior.
    resolved_request = self.linear_velocity_controller.resolve()
    request_name = resolved_request.source
    request_value = resolved_request.value
    resolved_priority = resolved_request.priority
```
---

## Quick Links

- [Overview (start here!)](docs/OVERVIEW.md)
- [Robot Guide](docs/features/ROBOT_GUIDE.md)
- [Interface Guide](docs/features/INTERFACE_GUIDE.md)
- [Requests Guide](docs/features/REQUESTS_GUIDE.md)
- [Faults Guide](docs/features/FAULTS_GUIDE.md)
- [Actions Guide](docs/features/ACTIONS_GUIDE.md)
- [Profiling Guide](docs/features/PROFILING_GUIDE.md)
- [SysID Guide](docs/features/SYSID_GUIDE.md)
- [Utils Guide](docs/features/UTILS_GUIDE.md)
- [Motivation](docs/MOTIVATION.md)

## Examples

- [Basic Component](examples/simple_component/README.md)
- [Drivetrain Robot](examples/drivetrain_robot/README.md)
- [Arm Robot](examples/arm_robot/README.md)
- [Competition Robot](examples/comp_tank_drive/README.md)

## Usage
1. Clone the repo (`git clone https://github.com/igowuu/AdaptiveRobot.git`) 
2. In the root directory, run `pip install -e .` to install the package in editable mode.
3. From `examples/full_tank_drive`, run `robotpy sim`.

**Developers:**

<div style="width:100px;">
  <a href="https://github.com/igowuu">
    <img width="100" height="100" src="https://github.com/user-attachments/assets/b507347a-6ba6-446e-be0a-7d2b0b26452e">
  </a>
  <div>
    <a>&nbsp;&nbsp;&nbsp;&nbsp;</a>
    <a href="https://github.com/igowuu">igowuu</a>
  </div>
</div>

---

## License

Licensed under the MIT License. See LICENSE file for details.

---
