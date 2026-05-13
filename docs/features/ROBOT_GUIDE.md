# Robot Lifecycle & Configuration

## Overview

`AdaptiveRobot` is the core of your robot code. It manages the robot's lifecycle, discovers your components, runs them each loop, and handles autonomous actions. Its an orchestrator that makes sure everything runs in the right order and at the correct time.

When you inherit from `AdaptiveRobot`, you get automatic scheduling, telemetry, fault handling, and action management without having to do it yourself.

## Quick Start

```python
from adaptive_robot import AdaptiveRobot

class MyRobot(AdaptiveRobot):
    def __init__(self) -> None:
        super().__init__()
        self.drivetrain = Drivetrain()  # Auto-discovered
        self.intake = Intake()          # Auto-discovered
    
    def onTeleopPeriodic(self) -> None:
        # This runs every loop during teleop
        pass
```

`AdaptiveRobot` will discover your components and run them automatically.

## Configuration

If you want to customize how the robot runs, pass a `RobotConfig` to the constructor:

```python
from adaptive_robot import AdaptiveRobot, RobotConfig

config = RobotConfig(
    period=0.02,                           # 50Hz (default)
    fault_logging_folder="faults/",        # Where to save fault logs
    default_fault_threshold=10,            # Max consecutive faults before unhealthy
    profiling_enabled=True,                # Profile method timing
    profile_logging_frequency=10,          # Save profiling data every 10 seconds
    telemetry_rounding_digits=5,           # Round telemetry to 5 decimals
    auto_discover_interfaces=True,         # Auto-discover components (recommended)
    verbose_discovery=False,               # Print discovery logs to console
)

class MyRobot(AdaptiveRobot):
    def __init__(self) -> None:
        super().__init__(config)
```

In most cases, the defaults are fine. Just remember to call `super().__init__()` (or `super().__init__(config)` if customizing).

## Lifecycle Methods

The robot goes through several lifecycle phases: initialization, periodic execution, and mode transitions. You override these methods to hook into specific points:

```python
class MyRobot(AdaptiveRobot):
    def onRobotInit(self) -> None:
        # Called once at startup
        self.drivetrain = Drivetrain()
        self.intake = Intake()
    
    def onRobotPeriodic(self) -> None:
        # Called every loop in all modes
        pass
    
    def onTeleopInit(self) -> None:
        # Called once when entering teleop
        pass
    
    def onTeleopPeriodic(self) -> None:
        # Called every loop during teleop
        pass
    
    # Similar patterns for onDisabledInit/Periodic, onAutonomousInit/Periodic, etc.
```

### Initialization & Every-Loop

| Method | Called | Use for |
|--------|--------|---------|
| `onRobotInit()` | Once at startup | Create components and initialize hardware |
| `onRobotPeriodic()` | Every loop, all modes | Code that runs regardless of mode |

### Disabled Mode

| Method | Called | Use for |
|--------|--------|---------|
| `onDisabledInit()` | Once when entering disabled | Cleanup and safe shutdown |
| `onDisabledPeriodic()` | Every loop while disabled | Display diagnostics, log data |
| `onDisabledExit()` | Once when leaving disabled | Prep for next mode (auto-saves tunables) |

### Teleop Mode

| Method | Called | Use for |
|--------|--------|---------|
| `onTeleopInit()` | Once when entering teleop | Initialize driver controls |
| `onTeleopPeriodic()` | Every loop during teleop | Read joystick, request subsystem actions |
| `onTeleopExit()` | Once when leaving teleop | Cleanup (motors stop automatically) |

### Autonomous Mode

| Method | Called | Use for |
|--------|--------|---------|
| `onAutonomousInit()` | Once when entering autonomous | Call `schedule_action()` to run your routine |
| `onAutonomousPeriodic()` | Every loop during autonomous | Typically left empty (actions run in background) |
| `onAutonomousExit()` | Once when leaving autonomous | Cleanup |

### Test Mode

| Method | Called | Use for |
|--------|--------|---------|
| `onTestInit()` | Once when entering test | Calibration, hardware testing |
| `onTestPeriodic()` | Every loop during test | Run test logic |
| `onTestExit()` | Once when leaving test | Cleanup |

---

## API Reference

### Registration

Use these for manual control. Otherwise, auto-discovery is simpler:

```python
# Register individual components by interface
robot.register_schedulable(obj)                      # Schedule each loop
robot.register_telemetry_publishable(obj)            # Publish telemetry
robot.register_tunable_publishable(obj)              # Manage tunables

# Register all three at once (AdaptiveComponent only)
robot.register_component(adaptive_component_obj)

# Unregister (before first loop)
robot.unregister_schedulable(obj)
robot.unregister_telemetry_publishable(obj)
robot.unregister_tunable_publishable(obj)
robot.unregister_component(adaptive_component_obj)
```

**Constraint:** All registration/unregistration must happen in `onRobotInit()` or earlier.

### Query Methods

Check what's registered or running:

```python
# Get lists of registered objects
schedulables = robot.get_registered_schedulables()
telemetry_objs = robot.get_registered_telemetry_publishables()
tunable_objs = robot.get_registered_tunable_publishables()

# Query actions
running = robot.get_running_actions()                # List of AsyncAction objects
is_running = robot.action_is_running("auto_name")   # True/False
```

### Action Management

Schedule and control long-running autonomous routines. See [ACTIONS_GUIDE.md](./ACTIONS_GUIDE.md) for writing actions:

```python
# Schedule an action (only one can run at a time)
def onAutonomousInit(self) -> None:
    name = self.schedule_action(my_autonomous(), "auto_drive")
    if name:
        print("Action started")
    else:
        print("Action already running")

# Check status
if self.action_is_running("auto_drive"):
    print("Still running")

# Cancel an action
self.cancel_action("auto_drive")

# Cancel everything
self.cancel_all_actions()
```

**Returns:** Action methods return the name on success, `None` on failure.

---

## Usage Examples

Patterns from main example (`examples/comp_tank_drive`).

### Example 1: Robot Init with Real vs Simulated IO

Create different IO depending on whether the robot is in simulation:

```python
class PerryV3(AdaptiveRobot):
    def __init__(self) -> None:
        super().__init__()
        
        self.controller = Joystick(0)
        
        # Choose IO based on simulation mode
        if self.isSimulation():
            self.drivetrain_io = SimulatedDrivetrainIO()
            self.shooter_io = SimulatedShooterIO()
        else:
            self.drivetrain_io = RealDrivetrainIO()
            self.shooter_io = RealShooterIO()
        
        # Components are auto-discovered
        self.drivetrain = Drivetrain(self.drivetrain_io)
        self.shooter = Shooter(self.shooter_io)
```

### Example 2: Controllers with Joystick Input

Controllers handle driver input and make requests on behalf of the driver:

```python
class DrivetrainController(Schedulable):
    def __init__(self, drivetrain: Drivetrain, controller: Joystick):
        self.drivetrain = drivetrain
        self.controller = controller
    
    def execute(self) -> None:
        if not RobotState.isTeleop():
            return
        
        linear = self.controller.getRawAxis(DRIVE_LINEAR)
        angular = self.controller.getRawAxis(DRIVE_ANGULAR)
        
        self.drivetrain.request_linear_velocity(
            linear * MAX_SPEED, 
            BasicPriority.TELEOP, 
            "teleop"
        )
```

### Example 3: Scheduling an Action from Teleop

Press a button to run an autonomous routine mid-teleop:

```python
def onTeleopPeriodic(self) -> None:
    if self.controller.getRawButtonPressed(JoystickButton.AIM_TO_HUB):
        self.schedule_action(point_to_hub(self.drivetrain), "point_to_hub")
```

### Example 4: Autonomous with ActionChooser

Let the driver select which autonomous to run:

```python
def __init__(self) -> None:
    super().__init__()
    
    self.action_chooser = ActionChooser()
    self.action_chooser.add_option("taxi_drive", lambda: taxi_drive(self.drivetrain))
    self.action_chooser.set_default("collect_balls", lambda: collect_balls(self.drivetrain))
    self.action_chooser.publish()

def onAutonomousInit(self) -> None:
    selected_action = self.action_chooser.get_selected_factory()
    selected_action_name = self.action_chooser.get_selected_name()
    self.schedule_action(selected_action(), selected_action_name)
```

### Example 5: Multiple Subsystem Controllers

Create separate controllers for each subsystem, all auto-discovered and run:

```python
def __init__(self) -> None:
    super().__init__()
    
    self.drivetrain = Drivetrain(io)
    self.intake = Intake(io)
    self.shooter = Shooter(io)
    
    # All these controllers are auto-discovered and scheduled
    self.drivetrain_controller = DrivetrainController(self.drivetrain, self.controller)
    self.intake_controller = IntakeController(self.intake, self.controller)
    self.shooter_controller = ShooterController(self.shooter, self.drivetrain, self.controller)
```

---

## Things to Remember

- **Registration is locked after the first loop.** Get all your components created and registered in `onRobotInit()` or `__init__`. 
- **Auto-discovery doesn't find components in lists/dicts.** Only attributes directly on `self`.
- **Only one action can run at a time.** If you try to schedule while one is already running, it returns `None`.
- **Tunables are auto-saved on exit.** When the robot disables, `onDisabledExit()` automatically calls `TunablePersistenceManager.save_all()`.
- **Actions are cancelled automatically in certain transitions.** `DisabledInit`, `AutonomousExit`, and `TestExit` automatically call `cancel_all_actions()`.
- **Use the table lookup above.** Each mode has specific auto-behaviors. Check the tables if you're unsure what's automatic vs what you need to do manually.

**Auto-behavior:** AdaptiveRobot cancels all running actions when exiting test.

---

## Component Discovery & Registration

By default, `AdaptiveRobot` automatically finds any components you create as instance variables. This is called **auto-discovery** and it's the recommended approach.

### Auto-Discovery (Recommended)

Just create your components in `onRobotInit()` and assign them to `self`:

```python
def onRobotInit(self) -> None:
    self.drivetrain = Drivetrain()      # Auto-discovered
    self.intake = Intake()              # Auto-discovered
    self.shooter = Shooter()            # Auto-discovered
```

AdaptiveRobot recursively searches your robot object and finds anything implementing `Schedulable`, `TelemetryPublishable`, or `TunablePublishable`. Nested interfaces will be found.

**When to disable auto-discovery:** If you store components in lists/dicts or want manual control, disable it via `RobotConfig(auto_discover_interfaces=False)` and manually register instead.

### Manual Registration

If you need fine-grained control or your components aren't directly attached to `self`, register them manually:

```python
def onRobotInit(self) -> None:
    self.custom_objects = [Drivetrain(), Intake()]
    
    # Manually register each
    for obj in self.custom_objects:
        if isinstance(obj, Schedulable):
            self.register_schedulable(obj)
        if isinstance(obj, TelemetryPublishable):
            self.register_telemetry_publishable(obj)
        if isinstance(obj, TunablePublishable):
            self.register_tunable_publishable(obj)
```

**Important:** All registration must happen in `__init__()` or `onRobotInit()`. Once the first loop runs, registration is locked.
