# AsyncAction & Autonomous Routines

## Overview

`AsyncAction` is a generator-based system for writing autonomous routines and long-running robot tasks. Instead of managing state machines manually with flags and counters, you write routines as sequential generator code. The framework handles scheduling, concurrency, cancellation, and error management.

This means autonomous routines read like normal Python - top to bottom - while still supporting parallel operations, racing, and timeouts. No callback hell, no state machine boilerplate.

## Quick Start

Create an autonomous routine as a generator function, use `yield from` to compose actions, then schedule it:

```python
from adaptive_robot import AdaptiveRobot, AsyncAction, wait, parallel

def autonomous_routine(robot: AdaptiveRobot) -> AsyncAction:
    # Sequential: runs step by step
    yield from robot.drivetrain.move_forward(distance=5)
    yield from wait(0.5)
    yield from robot.intake.spin_up()
    
    # Parallel: runs both at the same time
    yield from parallel(
        robot.drivetrain.move_forward(distance=5),
        robot.intake.collect_pieces()
    )

# Schedule it in autonomous init
class MyRobot(AdaptiveRobot):
    def onAutonomousInit(self) -> None:
        routine = autonomous_routine(self)
        self.schedule_action(routine, "auto_drive")
```

The scheduler runs your routine each iteration and manages everything else.

---

## Core Concepts

### AsyncAction Type

An `AsyncAction` is a Python generator function:

```python
AsyncAction = Generator[Instruction | None, None, Any]
```

It:
- **Uses `yield from`** to delegate to other generators (helper functions or component actions)
- **Yields `None`** or an `Instruction` when pausing internally
- **Completes** when the function returns (Python's implicit `StopIteration`)
- **Can be cancelled** at any time by the scheduler

**Example:**

```python
def simple_action() -> AsyncAction:
    print("Step 1")
    yield from wait(1.0)
    
    print("Step 2")
    yield from wait(1.0)
    
    print("Done!")
```

### Why Generators?

Traditional state machines require flags, counters, and lots of boilerplate:

```python
class OldAuto:
    def __init__(self):
        self.state = 0
        self.timer = 0
    
    def periodic(self):
        if self.state == 0:
            self.drivetrain.move(0.5)
            self.timer += 1
            if self.timer > 50:
                self.state = 1
                self.timer = 0
        elif self.state == 1:
            self.intake.spin(0.5)
            self.timer += 1
            if self.timer > 25:
                self.state = 2
        # ... more states
```

Generators let you write it sequentially:

```python
# New way (with AsyncAction)
def autonomous() -> AsyncAction:
    yield from drivetrain.move_forward(5)
    yield from intake.spin_up()
```

The framework handles scheduling and stepping `AsyncActions` internally.

---

## Helper Functions

These helper functions simplify writing common action patterns. All use `yield from` to delegate:

### `wait(duration: float) -> AsyncAction`

Pauses execution for a specified duration.

**Parameters:**
- `duration` - Time to wait in seconds (must be > 0)

**Example:**

```python
def timed_sequence(robot: AdaptiveRobot) -> AsyncAction:
    yield from robot.shooter.spin_up()
    yield from wait(0.5)  # Wait half a second
    yield from robot.shooter.fire()
```

### `parallel(*actions: AsyncAction) -> AsyncAction`

Runs multiple actions concurrently. Waits for all to complete.

**Parameters:**
- `*actions` - Variable number of action generators to run together

**Returns:** A generator that completes when all actions finish

**Example:**

```python
def multi_task(robot: AdaptiveRobot) -> AsyncAction:
    # All three run at the same time
    yield from parallel(
        robot.drivetrain.move_forward(5),
        robot.intake.collect_pieces(),
        robot.arm.raise_to_position(50)
    )
    
    print("All components ready")
```

**When to use:** Intake and drivetrain both need to run together, or multiple independent operations that don't need to sequence.

### `race(*actions: AsyncAction) -> AsyncAction`

Runs multiple actions concurrently. Returns when the first one finishes (others are cancelled).

**Parameters:**
- `*actions` - Variable number of action generators to race

**Returns:** A generator that completes on first completion

**Example:**

```python
def collect_with_timeout(robot: AdaptiveRobot) -> AsyncAction:
    # Whichever finishes first wins. The other is cancelled.
    yield from race(
        robot.intake.collect_until_full(),
        wait(5.0)  # 5-second timeout
    )
    
    print("Collected or timed out")
```

**When to use:** You want to wait for something to complete, but need a timeout as a safety net. Or you want to race two strategies and use whichever finishes first.

### `with_timeout(action: AsyncAction, timeout: float) -> AsyncAction`

Runs an action but cancels it if it takes too long.

**Parameters:**
- `action` - The action generator to run
- `timeout` - Maximum duration in seconds

**Returns:** A generator that runs the action with a timeout

**Example:**

```python
def time_limited_pathfind(robot: AdaptiveRobot) -> AsyncAction:
    yield from with_timeout(
        robot.pathfinder.find_optimal_path(),
        timeout=3.0
    )
    
    # If pathfinder took more than 3 seconds, it's cancelled here
    yield from robot.drivetrain.drive_path()
```

**When to use:** An action might take variable time, and you want to guarantee it doesn't block the rest of autonomous.

---

## Scheduling Actions

### Scheduling from `onAutonomousInit()`

The most common pattern is to schedule your autonomous routine when entering autonomous mode:

```python
class MyRobot(AdaptiveRobot):
    def onAutonomousInit(self) -> None:
        # Create and schedule the routine
        routine = autonomous_drive(self.drivetrain, self.intake)
        
        # Store the name, or check if it failed
        name = self.schedule_action(routine, "auto_drive")
        if name:
            print(f"Started action: {name}")
        else:
            print("Failed to start (another action running)")
```

### Scheduling from Teleop

You can also schedule actions during the teleop and test OpModes (for example, when a button is pressed):

```python
def onTeleopPeriodic(self) -> None:
    if self.controller.getRawButtonPressed(5):  # LB button
        # Start a mid-match autonomous routine
        self.schedule_action(align_to_target(self.drivetrain), "align")
```

### ActionScheduler API

Use these methods on your robot to manage actions:

| Method | Returns | Use for |
|--------|---------|---------|
| `schedule_action(action, name)` | `str` or `None` | Start a new action |
| `cancel_action(name)` | `str` or `None` | Stop a specific action |
| `cancel_all_actions()` | `list[str]` or `None` | Stop all actions |
| `action_is_running(name)` | `bool` | Check if an action is running |
| `get_running_actions()` | `list[str]` | Get names of all running actions |

**Example:**

```python
# Schedule
name = self.schedule_action(my_routine(), "routine_a")

# Check if running
if self.action_is_running("routine_a"):
    print("Still going")

# Cancel
self.cancel_action("routine_a")

# Get all running
running = self.get_running_actions()
for action_name in running:
    print(f"Running: {action_name}")
```

---

## Writing Custom Actions

### Basic Pattern

Actions are generator functions that use `yield from` to delegate to other generators:

```python
def my_action() -> AsyncAction:
    print("Starting action")
    yield from wait(1.0)
    print("Halfway through")
    yield from wait(1.0)
    print("Action complete")
```

### With Components

Pass components as parameters and request from them:

```python
def pickup_routine(drivetrain: Drivetrain, intake: Intake) -> AsyncAction:
    """Move to goal, intake, return home."""
    yield from drivetrain.move_to_goal(target_x=5, target_y=0)
    
    intake.spin()  # Start spinning (non-blocking)
    yield from wait(0.5)
    
    yield from intake.collect_until_full()
    yield from drivetrain.move_to_goal(target_x=0, target_y=0)
    
    print("Pickup complete!")
```

### With Conditional Logic

Actions can make decisions based on robot state or game data:

```python
def adaptive_auto(robot: AdaptiveRobot) -> AsyncAction:
    """Different strategy based on position."""
    if robot.is_left_side():
        yield from left_auto(robot)
    elif robot.is_right_side():
        yield from right_auto(robot)
    else:
        yield from center_auto(robot)
```

### With Loops

For repeated actions or retry logic:

```python
def collect_until_full(intake: Intake) -> AsyncAction:
    """Keep collecting until the intake reports full."""
    while not intake.is_full():
        yield from wait(0.1)  # Check every 100ms
    
    print("Intake full!")
```

### With Error Handling

Try/except works inside actions. Faults are logged automatically:

```python
def safe_auto(robot: AdaptiveRobot) -> AsyncAction:
    """Handle failures gracefully."""
    try:
        yield from robot.drivetrain.drive_path()
    except Exception as e:
        print(f"Drive failed: {e}")
        # Fallback behavior
        yield from robot.drivetrain.move_forward(3)
```

---

## Instruction System

Under the hood, actions communicate with the scheduler via **Instructions**. You rarely create these directly, but it's helpful to understand the flow.

### How It Works

1. Action yields an `Instruction`
2. Scheduler calls `instruction.step()` each iteration
3. Scheduler checks `instruction.is_complete()`
4. When complete, scheduler resumes the action

### Built-in Instructions

The helper functions create instructions automatically:

| Instruction | Created by | Purpose |
|-----------|-----------|---------|
| `WaitInstruction` | `wait()` | Pauses for a duration |
| `TimeoutInstruction` | `with_timeout()` | Wraps an action with a timeout |
| `RaceInstruction` | `race()` | Runs multiple actions, stops on first finish |
| `ParallelInstruction` | `parallel()` | Runs multiple actions, waits for all |

**Example of what happens internally:**

```python
def my_action() -> AsyncAction:
    yield from wait(0.5)  # Creates WaitInstruction(0.5)

# Scheduler does this:
# 1. Gets WaitInstruction from action
# 2. Each iteration: instruction.step()
# 3. Each iteration: checks is_complete()
# 4. When complete, resumes action
```

You typically don't create instructions yourself, but if you need custom behavior, you can inherit from `Instruction` in `adaptive_robot/autonomous/instructions.py` and implement the abstract methods.

---

## Common Patterns

### Sequential Routine

Run actions step by step (most common):

```python
def simple_auto(robot: AdaptiveRobot) -> AsyncAction:
    yield from robot.drivetrain.move_forward(5)
    yield from wait(0.5)
    yield from robot.intake.spin_up()
    yield from wait(1.0)
    yield from robot.intake.spin_down()
```

### Parallel Operations

Run multiple things at once (both must complete):

```python
def parallel_auto(robot: AdaptiveRobot) -> AsyncAction:
    # Drivetrain and intake run together
    yield from parallel(
        robot.drivetrain.move_forward(5),
        robot.intake.collect_pieces()
    )
    # Resumes here when BOTH are done
```

### Racing (First to Finish)

Run multiple things, continue when first completes:

```python
def race_auto(robot: AdaptiveRobot) -> AsyncAction:
    # Whichever finishes first wins
    yield from race(
        robot.intake.collect_until_full(),
        wait(5.0)  # 5-second timeout
    )
    # If intake filled: collect_until_full returned
    # If timeout: wait(5.0) returned
```

### Timeout Safety

Guarantee an action doesn't block the whole routine:

```python
def safe_pathfind(robot: AdaptiveRobot) -> AsyncAction:
    yield from with_timeout(
        robot.pathfinder.find_optimal_path(),
        timeout=2.0
    )
    # If pathfinder took > 2s, it's cancelled
    
    yield from robot.drivetrain.drive_path()
```

### Multi-Path Autonomous

Choose a strategy based on game state:

```python
def game_aware_auto(robot: AdaptiveRobot) -> AsyncAction:
    game_state = robot.get_game_data()
    
    if game_state.starting_position == Position.LEFT:
        yield from left_side_auto(robot)
    elif game_state.starting_position == Position.RIGHT:
        yield from right_side_auto(robot)
    else:
        yield from center_auto(robot)
```

### Retry Logic

Try an action multiple times before giving up:

```python
def retry_pickup(intake: Intake) -> AsyncAction:
    max_attempts = 3
    
    for attempt in range(max_attempts):
        try:
            yield from intake.collect_until_full()
            break  # Success, exit loop
        except Exception:
            if attempt < max_attempts - 1:
                yield from wait(0.5)  # Wait before retry
```

---

## Action Execution Flow

### Lifecycle

Here's what happens when you schedule an action:

```
schedule_action(action, name)
    ↓
Action added to scheduler
    ↓
Each robot iteration:
    1. Call action.send(None) - resumes generator
    2. Action yields Instruction (or None)
    3. Scheduler calls instruction.step()
    4. Scheduler checks instruction.is_complete()
    5. If complete:
       - Call instruction.on_complete()
       - Resume action to next yield
    6. If incomplete:
       - Store instruction, continue next iteration
    ↓
Action returns (Python's StopIteration)
    ↓
Action removed from scheduler
```

### Cancellation Flow

When you cancel an action:

```
cancel_action(name)
    ↓
Get current instruction
    ↓
Call instruction.cleanup()
    ↓
Close action generator
    ↓
Action removed from scheduler
```

### Error Handling

All exceptions in actions are caught and logged as CRITICAL faults:

- **Action raises exception** → Logged, action cancelled
- **Instruction.step() raises exception** → Logged, action cancelled
- **Instruction.cleanup() raises exception** → Logged (action still removed)

This means your robot won't crash if an action fails. The fault is logged to disk so you can review it later.

---

## Real-World Examples

### Example 1: Simple Taxi (comp_tank_drive)

Move the robot a few meters forward:

```python
from adaptive_robot import AsyncAction

def taxi_drive(drivetrain: Drivetrain) -> AsyncAction:
    """Drives the robot forward for a few meters."""
    yield from follow_trajectory(drivetrain, "taxi_drive")
```

### Example 2: Arm Routine (arm_robot)

Move an arm up and down with waits:

```python
def demo_auto(arm: Arm) -> AsyncAction:
    """Move arm up, wait, move arm down."""
    
    # Move to max angle
    while True:
        angle_error = ArmConstants.MAX_ANGLE - arm.get_position()
        if angle_error < ArmConstants.ANGLE_TOLERANCE:
            break
        else:
            arm.request_angle(ArmConstants.MAX_ANGLE, BasicPriority.AUTO, "auto")
        yield
    
    yield from wait(0.5)  # Wait half second
    
    # Move to min angle
    while True:
        angle_error = arm.get_position() - ArmConstants.MIN_ANGLE
        if angle_error < ArmConstants.ANGLE_TOLERANCE:
            break
        else:
            arm.request_angle(ArmConstants.MIN_ANGLE, BasicPriority.AUTO, "auto")
        yield
```

### Example 3: Action Chooser (comp_tank_drive)

Let the driver select which autonomous to run:

```python
class PerryV3(AdaptiveRobot):
    def __init__(self) -> None:
        super().__init__()
        
        # Create action chooser
        self.action_chooser = ActionChooser()
        self.action_chooser.add_option("taxi_drive", lambda: taxi_drive(self.drivetrain))
        self.action_chooser.add_option("collect_and_shoot", lambda: collect_balls(self.drivetrain, self.shooter))
        self.action_chooser.set_default("taxi_drive")
        self.action_chooser.publish()
    
    def onAutonomousInit(self) -> None:
        # Get the selected routine
        selected_factory = self.action_chooser.get_selected_factory()
        selected_name = self.action_chooser.get_selected_name()
        
        # Schedule it
        self.schedule_action(selected_factory(), selected_name)
```

---

## Best Practices

### 1. Keep Actions Readable

Write sequentially first, then add concurrency if needed:

```python
# Good - reads top to bottom
def auto() -> AsyncAction:
    yield from drivetrain.move(5)
    yield from intake.spin()
    yield from wait(1)

# Harder to follow
def auto() -> AsyncAction:
    yield from parallel(
        race(drivetrain.move(5), wait(2)),
        parallel(intake.spin(), arm.move())
    )
```

### 2. Extract Complex Subroutines

Break large actions into smaller, reusable ones:

```python
# Good - reusable pieces
def collect_routine() -> AsyncAction:
    yield from intake.spin_up()
    yield from wait(0.5)

def shoot_routine() -> AsyncAction:
    yield from shooter.spin_up()
    yield from wait(0.2)
    yield from shooter.fire()

def collect_and_shoot() -> AsyncAction:
    yield from collect_routine()
    yield from shoot_routine()

# Instead of one giant function
```

### 3. Use Timeouts for Safety

Always add timeouts to actions that might hang:

```python
# Good - has timeout
yield from with_timeout(
    arm.move_to_position(target),
    timeout=3.0
)

# Risky - might block forever
yield from arm.move_to_position(target)
```

### 4. Log Important State

Print key milestones so you can debug from logs:

```python
def auto() -> AsyncAction:
    print("Starting autonomous")
    yield from drivetrain.move(5)
    print("Drive complete, starting intake")
    yield from intake.collect()
    print("Autonomous finished")
```

### 5. Use Parallel When Things Don't Depend

Parallel is faster than sequential:

```python
# Good - shooter and drivetrain are independent
yield from parallel(
    drivetrain.move_to_goal(),
    shooter.spin_up()
)

# Slower - waits for drivetrain before spinning shooter
yield from drivetrain.move_to_goal()
yield from shooter.spin_up()
```

---

## Troubleshooting

### Action Doesn't Start

**Problem:** `schedule_action()` returns `None`

**Cause:** Another action is already running (only one primary autonomous action can run at a time)

**Solution:**
```python
if self.schedule_action(routine, "name") is None:
    print("Failed to start - cancel running action first")
    self.cancel_action("other_name")
```

### Action Runs But Doesn't Finish

**Problem:** Action seems stuck or hangs

**Cause:** Missing `yield` or infinite loop without timeouts

**Solution:**
```python
# Bad - no yield, blocks forever
def bad_auto() -> AsyncAction:
    while not done():
        pass  # No yield here!

# Good - yields to let scheduler run
def good_auto() -> AsyncAction:
    while not done():
        yield  # Doesn't block the scheduler from running
```

### Action Cancelled Without Warning

**Problem:** Action is cancelled unexpectedly

**Cause:** An exception was raised and logged as a fault

**Solution:** Check the fault logs:
```python
# Faults are saved to disk (check fault_logging_folder in RobotConfig)
# Each fault includes timestamp, description, and traceback
```

### Parallel Doesn't Wait for All

**Problem:** Parallel returns before all actions finish

**Cause:** Using `yield from race()` instead of `yield from parallel()`

**Solution:**
```python
# Good - waits for both
yield from parallel(action1(), action2())

# Wrong - stops on first
yield from race(action1(), action2())
```

---

## See Also

- [Robot Lifecycle & Configuration](./ROBOT_GUIDE.md) - How to structure your robot class
- [Request Arbitration](./REQUESTS_GUIDE.md) - How to handle competing requests
- [Fault Handling](./FAULTS_GUIDE.md) - How to detect and handle failures
- [Interfaces & Components](./INTERFACE_GUIDE.md) - How to structure your subsystems
- [Action Chooser Example](../../examples/comp_tank_drive/) - Real multi-action autonomous
