# AdaptiveRobot Framework

> Autonomous, telemetry, safety, and interface-oriented control.

## Quick Links

- [Overview (start here!)](docs/OVERVIEW.md)
- [Robot Guide](docs/features/ROBOT_GUIDE.md)
- [Interface Guide](docs/features/INTERFACE_GUIDE.md)
- [Requests Guide](docs/features/REQUESTS_GUIDE.md)
- [Faults Guide](docs/features/FAULTS_GUIDE.md)
- [Actions Guide](docs/features/ACTIONS_GUIDE.md)
- [Motivation](docs/MOTIVATION.md)
- [Future](docs/FUTURE.md)

## Examples

- [Basic Component](examples/simple_component/README.md)
- [Drivetrain Robot](examples/drivetrain_robot/README.md)
- [Arm Robot](examples/arm_robot/README.md)
- [Competition Robot](examples/comp_tank_drive/README.md)

## Usage
1. Clone the repo (`git clone https://github.com/igowuu/AdaptiveRobot.git`) 
2. In the root directory, run `pip install -e .` to install the package in editable mode.
3. From `examples/full_tank_drive`, run `robotpy sim`.

## Overview

AdaptiveRobot is an alternative python framework for building FRC robot code, based upon
- **Interfaces and Components** - Allows any class to communicate with NT, publish tunables, faults, and access lifecycle methods.
- **Telemetry** - Built-in dashboard telemetry and logging
- **Tuning** - Live parameter adjustment with NetworkTables
- **Faults** - Fault detection, logging, component health
- **Autonomous** - Generator-based asynchronous command sequences

---

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
