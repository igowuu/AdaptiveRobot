# AdaptiveRobot Framework
## Quick Links

- [Robot Guide](docs/features/ROBOT_GUIDE.md)
- [Components Guide](docs/features/COMPONENTS_GUIDE.md)
- [Requests Guide](docs/features//REQUESTS_GUIDE.md)
- [Faults Guide](docs/features/FAULTS_GUIDE.md)
- [Actions Guide](docs/features/ACTIONS_GUIDE.md)
- [Motivation](docs/MOTIVATION.md)
- [Future](docs/FUTURE.md)

## Usage
1. Clone the repo (`git clone https://github.com/igowuu/AdaptiveRobot.git`) 
2. In the root directory, run `pip install -e .` to install the package in editable mode.
3. From `examples/full_tank_drive`, run `robotpy sim`.

## Overview

AdaptiveRobot is an alternative python framework for building FRC robot code with:
- **Components** - Components attatched to the robot scheduler with lifecycle hooks
- **Telemetry** - Built-in dashboard telemetry and logging
- **Tuning** - Live parameter adjustment with NetworkTables
- **Faults** - Fault detection, logging, component health
- **Autonomous** - Generator-based asynchronous command sequences

A breif public polify note: This is a BETA project, the API is subject to unannounced breaking changes untill 1.0, after that we will not change the API mid season, and when we do it will be with at least a migration guide, preferable an automatic migrator.
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
<div style="width:100px;">
  <a href="https://github.com/chaseyalon">
    <img width="100" height="100" src="https://avatars.githubusercontent.com/u/149000633?s=400&v=4">
  </a>
  <div>
    <a>&nbsp;&nbsp;&nbsp;&nbsp;</a>
    <a href="https://github.com/chaseyalon">ChaseYalon</a>
  </div>
</div>
---

## License

Licensed under the MIT License. See LICENSE file for details.

---
