# Action Chooser

## Overview

Autonomous routines should be selectable. When testing different strategies, different starting positions, different game states, you need to let the operator easily choose which autonomous to run.

`ActionChooser` makes this simple. It's a wrapper around WPILib's `SendableChooser` that manages multiple autonomous routines and publishes them to SmartDashboard. The driver selects which action to run, and you get it programmatically in your robot code.

This is the standard FRC pattern for autonomous selection, adapted for AdaptiveRobot's async actions.

## Quick Start

Create an `ActionChooser`, add your autonomous routines, and publish it:

```python
from adaptive_robot import AdaptiveRobot, ActionChooser

def taxi_auto(drivetrain) -> AsyncAction:
    """Simple taxi routine."""
    yield from drivetrain.move_forward(3)

def score_auto(robot) -> AsyncAction:
    """Complex scoring routine."""
    yield from robot.intake.collect()
    yield from robot.shooter.fire()

class MyRobot(AdaptiveRobot):
    def onRobotInit(self) -> None:
        self.action_chooser = ActionChooser()
        
        # Add options
        self.action_chooser.add_option("Taxi", lambda: taxi_auto(self.drivetrain))
        self.action_chooser.add_option("Score", lambda: score_auto(self))
        
        # Set default
        self.action_chooser.set_default("Taxi", lambda: taxi_auto(self.drivetrain))
        
        # Publish to SmartDashboard
        self.action_chooser.publish()
    
    def onAutonomousInit(self) -> None:
        # Get selected routine and schedule it
        selected_factory = self.action_chooser.get_selected_factory()
        selected_name = self.action_chooser.get_selected_name()
        
        self.schedule_action(selected_factory(), selected_name)
```

That's it. When the match starts, the driver's selection from SmartDashboard is used.

---

## Core Concepts

### Action Factories

`ActionChooser` doesn't store actions directly - it stores **factories** (lambda functions that create actions).

Why? Because each time you select an action, you need a fresh copy. If autonomous runs twice, you need two separate action generators.

```python
# Good - factory returns a new action each time
self.action_chooser.add_option("Taxi", lambda: taxi_auto(self.drivetrain))

# Bad - stores single action, not reusable
action = taxi_auto(self.drivetrain)
self.action_chooser.add_option("Taxi", lambda: action)
```

### SmartDashboard Integration

`ActionChooser` publishes to SmartDashboard automatically via `SendableChooser`. The driver sees a dropdown menu and selects which action to run.

---

## API Reference

### Creating a Chooser

```python
from adaptive_robot import ActionChooser

chooser = ActionChooser()
```

Creates an empty chooser ready for options.

### Adding Options

#### `add_option(name: str, factory: ActionFactory) -> None`

Adds a selectable autonomous routine.

**Parameters:**
- `name` - Display name on SmartDashboard (e.g., "Taxi Drive")
- `factory` - Lambda that creates a new `AsyncAction` each time it's called

**Raises:** `ValueError` if the name already exists

**Example:**

```python
chooser.add_option("Taxi", lambda: taxi_auto(self.drivetrain))
chooser.add_option("Score", lambda: score_auto(self))
chooser.add_option("Do Nothing", lambda: wait(15))
```

### Setting Default

#### `set_default(name: str, factory: ActionFactory) -> None`

Sets which option is selected by default. If the option doesn't exist, it's added first.

**Parameters:**
- `name` - Name of the default option
- `factory` - Lambda that creates the action

**Example:**

```python
# Add as default (if not already added)
chooser.set_default("Taxi", lambda: taxi_auto(self.drivetrain))

# Or set default for existing option
chooser.add_option("Taxi", lambda: taxi_auto(self.drivetrain))
chooser.set_default("Taxi", lambda: taxi_auto(self.drivetrain))
```

### Getting Selection

#### `get_selected_factory() -> ActionFactory`

Returns the factory for the currently selected action.

**Returns:** A callable that creates the selected `AsyncAction`

**Raises:** `RuntimeError` if no selection and no default is set

**Example:**

```python
def onAutonomousInit(self) -> None:
    selected_factory = self.action_chooser.get_selected_factory()
    action = selected_factory()  # Create the action
    self.schedule_action(action, "auto")
```

#### `get_selected_name() -> str`

Returns the name of the currently selected action.

**Returns:** The name string

**Raises:** `RuntimeError` if no selection and no default is set

**Example:**

```python
name = self.action_chooser.get_selected_name()
print(f"Running autonomous: {name}")
```

### Querying Options

#### `get_available_options() -> list[str]`

Returns all registered option names.

**Returns:** List of option names

**Example:**

```python
options = self.action_chooser.get_available_options()
print(f"Available autos: {options}")  # ['Taxi', 'Score', 'Do Nothing']
```

### Publishing to Dashboard

#### `publish(key: str = "Auto") -> None`

Publishes the chooser to SmartDashboard.

**Parameters:**
- `key` - SmartDashboard key (default: `"Auto"`)

**Example:**

```python
# Publish under "Auto" (default)
self.action_chooser.publish()

# Or custom key
self.action_chooser.publish("SelectedAutonomous")
```

---

## Usage

### Multiple Autonomous Strategies

```python
class MyRobot(AdaptiveRobot):
    def onRobotInit(self) -> None:
        self.action_chooser = ActionChooser()
        
        # Left side autos
        self.action_chooser.add_option(
            "Left - Taxi",
            lambda: left_taxi_auto(self.drivetrain)
        )
        self.action_chooser.add_option(
            "Left - Score",
            lambda: left_score_auto(self.drivetrain, self.intake)
        )
        
        # Right side autos
        self.action_chooser.add_option(
            "Right - Taxi",
            lambda: right_taxi_auto(self.drivetrain)
        )
        self.action_chooser.add_option(
            "Right - Score",
            lambda: right_score_auto(self.drivetrain, self.intake)
        )
        
        self.action_chooser.publish()
    
    def onAutonomousInit(self) -> None:
        selected_factory = self.action_chooser.get_selected_factory()
        selected_name = self.action_chooser.get_selected_name()

        self.schedule_action(selected_factory(), selected_name)
```

### Dynamic Naming

```python
positions = ["Left", "Center", "Right"]
strategies = ["Taxi", "Score", "Balanced"]

for position in positions:
    for strategy in strategies:
        name = f"{position} {strategy}"
        
        # Create appropriate autonomous for this combination
        factory = lambda p=position, s=strategy: get_auto(p, s)
        self.action_chooser.add_option(name, factory)

# Set a reasonable default
self.action_chooser.set_default("Center Taxi", lambda: get_auto("Center", "Taxi"))
self.action_chooser.publish()
```

## Best Practices

### 1. Always Set a Default

If no default is set and the driver doesn't select anything, `get_selected_factory()` throws an exception.

```python
# Good - always has a safe default
self.action_chooser.set_default("Safe Taxi", lambda: taxi_auto(self.drivetrain))

# Risky - will crash if driver doesn't select
# (no set_default called)
```

### 2. Use Descriptive Names

Names should tell the driver what will happen:

```python
# Good
"Left Start - Collect & Score"
"Right Start - Taxi 5m"
"Center - Balanced"

# Confusing
"Auto 1"
"Strategy A"
"Do stuff"
```

### 3. Use Lambdas for Factories

Lambdas capture the robot state at call time, so each action gets fresh components:

```python
# Good - new action each time
self.action_chooser.add_option("Score", lambda: score_auto(self.drivetrain, self.intake))

# Problem - action is created once, reused
action = score_auto(self.drivetrain, self.intake)
self.action_chooser.add_option("Score", lambda: action)
```

### 4. Test All Options

Make sure every action in the chooser actually works:

```python
# During testing, cycle through all options
for option_name in self.action_chooser.get_available_options():
    # Manually select and test each one
    print(f"Testing {option_name}...")
```

---

## Troubleshooting

### No Options Appear on SmartDashboard

**Problem:** Chooser not showing on dashboard

**Cause:** `publish()` wasn't called, or called too early

**Solution:**
```python
def onRobotInit(self) -> None:
    self.action_chooser = ActionChooser()
    self.action_chooser.add_option("Taxi", lambda: taxi_auto(self))
    self.action_chooser.publish()  # Must call this!
```

### Selected Option Throws Exception

**Problem:** `get_selected_factory()` or `get_selected_name()` crashes

**Cause:** No default was set and driver didn't select anything

**Solution:**
```python
# Always set a default
self.action_chooser.set_default("Safe Option", lambda: safe_auto(self))
```

### Action Factory Returns Wrong Type

**Problem:** Chooser expects `AsyncAction` but factory returns something else

**Cause:** Factory isn't creating a generator

**Solution:**
```python
# Good - factory creates AsyncAction
self.action_chooser.add_option("Auto", lambda: auto_routine(self))  # Returns AsyncAction

# Bad - factory returns non-generator
self.action_chooser.add_option("Auto", auto_routine)  # Missing lambda!
```

---

## See Also

- [AsyncAction & Autonomous Routines](./ACTIONS_GUIDE.md) - How to write autonomous actions
- [Robot Lifecycle & Configuration](./ROBOT_GUIDE.md) - How to structure robot class
- [comp_tank_drive example](../../examples/comp_tank_drive/robot.py) - Real-world ActionChooser usage
