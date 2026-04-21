# AdaptiveRobot

## Overview
AdaptiveRobot is the main controller of a robot codebase. It executes lifecycle methods based on robot state and schedules, cancels, and executes Actions each iteration.

## Quick Start

Inherit from AdaptiveRobot and override lifecycle methods:

```python
from adaptive_robot.adaptive_robot import AdaptiveRobot

class MyRobot(AdaptiveRobot):
    def __init__(self) -> None:
        super().__init__()
    
    def onRobotInit(self) -> None:
        # Initialize components
        pass
    
    def onTeleopPeriodic(self) -> None:
        # Teleop (controller) control
        pass
```

## Parent Constructor

#### `__init__(period: seconds = 0.02, fault_logging_folder: str = "faults/") -> None`

**Parameters:**
- `period`: Time it takes between iterations in seconds (default: 0.02s = 50Hz)
- `fault_logging_folder`: Directory to save JSON fault logs (default: "faults/")

## Usage
- Must call `super().__init__()` from subclass
- Components are automatically discovered and registered during `robotInit()`
- If you manually create components before or in `robotInit()`, they'll be auto-discovered

## Lifecycle methods

Lifecycle methods use the "on_" naming convention and are overrideable hooks for user code.

> Framework-internal lifecycle methods (without "on_" prefix) are marked `@final` and handle main scheduling logic. Do not override those.

### Robot Initialization and Periodic

#### `onRobotInit() -> None`
- **Called:** Exactly once when the robot is first powered on
- **Purpose:** Robot-wide initialization (hardware setup, component creation)
- **Example:**
```python
def onRobotInit(self) -> None:
    self.drivetrain = DrivetrainComponent()  # Auto-discovered
    self.intake = IntakeComponent()          # Auto-discovered
```

#### `onRobotPeriodic() -> None`
- **Called:** Every robot iteration (50Hz by default)
- **Purpose:** Code that should run in all robot modes
- **Example:**
```python
def onRobotPeriodic(self) -> None:
    self.foo()  # Called every iteration in the robot loop
```

### Mode Transitions

All mode transitions follow the pattern: `Init` -> `Periodic` -> `Exit`

#### Disabled Mode

##### `onDisabledInit() -> None`
- **Called:** When robot enters disabled mode
- **Auto-behavior:** Cancels all running actions and resets all components to HEALTHY
- **Purpose:** Cleanup code for disabled mode

##### `onDisabledPeriodic() -> None`
- **Called:** Each iteration while robot is disabled
- **Purpose:** Disabled-mode-specific code

##### `onDisabledExit() -> None`
- **Called:** When robot leaves disabled mode
- **Purpose:** Preparation for next mode

#### Teleop Mode

##### `onTeleopInit() -> None`
- **Called:** When robot enters teleop mode
- **Purpose:** Setup for driver control

##### `onTeleopPeriodic() -> None`
- **Called:** Each iteration during teleop
- **Purpose:** Handle driver input and teleoperated robot behavior

##### `onTeleopExit() -> None`
- **Called:** When robot exits teleop mode
- **Purpose:** Cleanup relating to teleoperated mode

#### Autonomous Mode

##### `onAutonomousInit() -> None`
- **Called:** When robot enters autonomous mode
- **Auto-behavior:** None (use `schedule_action()` to run autonomous routines)
- **Purpose:** Setup for autonomous

##### `onAutonomousPeriodic() -> None`
- **Called:** Each iteration during autonomous
- **Purpose:** Autonomous-mode-specific code

##### `onAutonomousExit() -> None`
- **Called:** When robot exits autonomous mode
- **Auto-behavior:** Cancels all running actions
- **Purpose:** Cleanup after autonomous

#### Test Mode

##### `onTestInit() -> None`
- **Called:** When robot enters test mode
- **Purpose:** Setup for testing

##### `onTestPeriodic() -> None`
- **Called:** Each iteration during test mode
- **Purpose:** Test-mode-specific code

##### `onTestExit() -> None`
- **Called:** When robot exits test mode
- **Auto-behavior:** Cancels all running actions
- **Purpose:** Cleanup after testing

---

## Component Management

### Auto-Discovery
By default, components created as instance variables in `onRobotInit()` are automatically discovered and scheduled:

```python
def onRobotInit(self):
    self.drivetrain = Drivetrain()   # Auto-discovered
    self.intake = Intake()           # Auto-discovered
```

##### `register_component(component: AdaptiveComponent) -> None`

Manually registers a component to be scheduled.

**Parameters:**
- `component`: The AdaptiveComponent to register

**Returns:** None

**Constraints:**
- Must be called in `__init__()` or `onRobotInit()` (before first `robotPeriodic()`)
- Raises RuntimeError if component is already registered

**Example:**
```python
def __init__(self):
    super().__init__()
    self.drivetrain = Drivetrain()
    self.register_component(self.drivetrain)
```

##### `unregister_component(component: AdaptiveComponent) -> None`

Unregisters a component from being scheduled.

**Parameters:**
- `component`: The AdaptiveComponent to unregister

**Returns:** None

**Constraints:**
- Must be called in `__init__()` or `onRobotInit()` (before first `robotPeriodic()`)
- Ignored if component is not registered

---

## Action Scheduling

Actions are asynchronous tasks (typically autonomous sequences) managed by the robot. See [ACTIONS_GUIDE.md](./ACTIONS_GUIDE.md) for details on creating actions.

##### `schedule_action(action: AsyncAction, name: str) -> str | None`

Schedules an AsyncAction to run with the given name.

**Parameters:**
- `action`: The AsyncAction to schedule
- `name`: Unique identifier for this scheduled action

**Returns:**
- `name` if successfully scheduled
- `None` if the action was already running (duplicate scheduling prevented)

**Example:**
```python
def onAutonomousInit(self) -> None:
    auto_routine = AutonomousRoutine()
    if self.schedule_action(auto_routine, "auto_drive"):
        print("Autonomous started")
    else:
        print("Action already running!")
```

##### `cancel_action(name: str) -> str | None`

Cancels a running AsyncAction.

**Parameters:**
- `name`: Name of the action to cancel

**Returns:**
- `name` if successfully cancelled
- `None` if the action was not found

**Example:**
```python
def onTeleopPeriodic(self) -> None:
    if self.controller.getAButton():
        result = self.cancel_action("auto_drive")
        if result:
            print("Autonomous cancelled")
```

##### `cancel_all_actions() -> list[str] | None`

Cancels all running AsyncActions.

**Returns:**
- List of cancelled action names if any were cancelled
- `None` if no actions were running

**Auto-Called:** Automatically called when entering disabled mode or exiting autonomous/test modes

**Example:**
```python
def emergency_stop(self) -> None:
    cancelled = self.cancel_all_actions()
    if cancelled:
        print(f"Cancelled: {cancelled}")
```

---
