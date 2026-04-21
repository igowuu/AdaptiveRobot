# AsyncAction & Autonomous Routines

## Overview

`AsyncAction` is a generator-based system for writing autonomous routines. Instead of managing state machines manually, you write routines as generator code. The framework handles scheduling, cancellation, and error management.

This allows multi-step autonomous groups and singular actions to use native python syntax instead of a verbose scheduler.

## Quick Start

Create an autonomous routine as a generator function and use `yield from` to compose actions:

```python
from adaptive_robot import AdaptiveRobot, AsyncAction, wait, parallel, race, with_timeout

def autonomous_routine(robot: AdaptiveRobot) -> AsyncAction:
    # Everything in the routine will move sequentially unless wrapped
    yield from robot.drivetrain.move_forward(distance=5)
    yield from wait(0.5)
    yield from robot.intake.spin_up()
    
    # Run two things at the same time (until both finish)
    yield from parallel(
        robot.drivetrain.move_forward(distance=5),
        robot.intake.collect_pieces()
    )
    
    # Race two actions (continues once on finishes)
    yield from race(
        robot.arm.move_to_target(),
        wait(2.0)
    )

# Schedule it in your robot
class MyRobot(AdaptiveRobot):
    def onAutonomousInit(self) -> None:
        routine = autonomous_routine(self)
        self.schedule_action(routine, "auto_drive")
```

---

## AsyncAction Type

### Generator-Based Design

```python
AsyncAction = Generator[Instruction | None, None, Any]
```

An `AsyncAction` is a Python generator function that:
- **Uses `yield from`** to delegate to other generators (helper functions or component actions)
- **Yields `None`** or an `Instruction` to pause/wait
- **Completes** when the function returns (via implicit `StopIteration`)
- **Can be cancelled** at any time by the scheduler

**Example:**
```python
def simple_action() -> AsyncAction:
    print("Step 1")
    yield from wait(1.0)
    
    print("Step 2")
    yield from wait(1.0)
    
    print("Done!")

routine = simple_action()
robot.schedule_action(routine, "my_action")
```

---

## Helper Functions

These helper functions return generators that simplify writing common action patterns. Use `yield from` to delegate to them.

### `wait(duration: float) -> AsyncAction`

Pauses execution for a specified duration.

**Parameters:**
- `duration`: Time to wait in seconds (must be > 0)

**Example:**
```python
def timed_sequence(robot):
    yield from robot.shooter.spin_up()
    yield from wait(0.5)
    yield from robot.shooter.fire()
    yield from wait(2.0)
```

### `with_timeout(action: AsyncAction, timeout: float) -> AsyncAction`

Runs an action but cancels it if it takes too long.

**Parameters:**
- `action`: The action generator to run
- `timeout`: Maximum duration in seconds

**Returns:** A generator that runs the action with timeout

**Example:**
```python
def auto_with_safety(robot):
    yield from with_timeout(
        robot.pathfinder.find_optimal_path(),
        timeout=3.0
    )
```

### `parallel(*actions: AsyncAction) -> AsyncAction`

Runs multiple actions concurrently. Waits for all to complete.

**Parameters:**
- `*actions`: Variable number of action generators to run in parallel

**Returns:** A generator that runs all actions concurrently

**Example:**
```python
def multi_task_sequence(robot):
    yield from parallel(
        robot.drivetrain.move_forward(5),
        robot.intake.spin(),
        robot.arm.raise_to_position(50)
    )

    print("All components ready")
```

### `race(*actions: AsyncAction) -> AsyncAction`

Runs multiple actions concurrently. Returns when first one finishes (others are cancelled).

**Parameters:**
- `*actions`: Variable number of action generators to race

**Returns:** A generator that returns on first completion

**Example:**
```python
def race_example(robot):
    # Whichever finishes first wins. The others are cancelled.
    yield from race(
        robot.intake.collect_until_full(),
        wait(5.0)
    )
```

---

## Instruction System

The instruction system gives the action subscheduler control over action behavior. You can subclass Instruction if you wish to create custom behavior.

### Instruction Base Class

```python
class Instruction(ABC):
    def is_complete(self) -> bool:
        """
        Checks if instruction should finish
        """
        pass
    
    def step(self) -> None:
        """
        Called each iteration while running
        """
        pass
    
    def on_complete(self) -> None:
        """
        Called when instruction finishes
        """
        pass
    
    def cleanup(self) -> None:
        """
        Clean up resources (called on cancellation)
        """
        pass
```

### Built-in Instructions

#### WaitInstruction

Pauses for a duration.

```python
instruction = WaitInstruction(duration=2.0)
while not instruction.is_complete():
    instruction.step()
```

#### TimeoutInstruction

Wraps an action with a maximum duration.

```python
inner_action = some_movement()
instruction = TimeoutInstruction(inner_action, timeout=3.0)
```

#### RaceInstruction

Runs multiple actions, stops on first completion.

```python
instruction = RaceInstruction(
    action1,
    action2,
    action3
)
```

#### ParallelInstruction

Runs multiple actions, waits for all completion.

```python
instruction = ParallelInstruction(
    action1,
    action2,
    action3
)
```

---

## ActionScheduler

The `ActionScheduler` manages action execution and lifecycle.

### Scheduling Actions

#### `schedule(action: AsyncAction, name: str) -> str | None`

Schedules an action to run.

**Parameters:**
- `action`: The AsyncAction to schedule
- `name`: Unique name for this action

**Returns:**
- The action name if successfully scheduled
- `None` if an action with that name is already running

**Example:**
```python
def onAutonomousInit(self) -> None:
    auto_routine = autonomous_drive()
    result = self.schedule_action(auto_routine, "auto_drive")
    
    if result:
        print("Autonomous started")
    else:
        print("Action already running")
```

### Cancelling Actions

#### `cancel(name: str) -> str | None`

Cancels a specific running action.

**Parameters:**
- `name`: Name of the action to cancel

**Returns:**
- The action name if successfully cancelled
- `None` if the action was not found

**Example:**
```python
def onTeleopInit(self) -> None:
    if self.cancel_action("auto_drive"):
        print("Autonomous cancelled")
```

#### `cancel_all() -> list[str] | None`

Cancels all running actions.

**Returns:**
- List of cancelled action names
- `None` if no actions were running

**Example:**
```python
def emergency_stop(self) -> None:
    cancelled = self.cancel_all_actions()
    if cancelled:
        print(f"Cancelled: {cancelled}")
```

### Querying Actions

#### `is_running(name: str) -> bool`

Checks if an action is currently running.

**Example:**
```python
if self.is_running("auto_drive"):
    print("Autonomous routine still executing")
```

#### `get_running_actions() -> list[AsyncAction]`

Returns all currently running actions.

**Example:**
```python
active_actions = self.get_running_actions()
print(f"Running {len(active_actions)} actions")
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

### With Component Integration

```python
def pickup_routine(drivetrain: Drivetrain, intake: Intake) -> AsyncAction:
    yield from drivetrain.move_to_goal(target_x=5, target_y=0)
    intake.spin()
    yield from wait(0.5)
    yield from intake.collect_until_full()
    yield from drivetrain.move_to_goal(target_x=0, target_y=0)
```

### With Timeouts

```python
def time_limited_action(arm: Arm) -> AsyncAction:
    yield from with_timeout(
        arm.move_to_position(target=100),
        timeout=3.0
    )

    print("Action done or timed out")
```

### With Component Locking

Special actions can lock components to bypass normal scheduling:

```python
def sysid_test(drivetrain: Drivetrain, drivetrain_io: DrivetrainIO, test: SysIdTest) -> AsyncAction:
    try:
        robot.drivetrain.locked = True
        for _ in test.run():
            robot.io.update()
            yield
    finally:
        robot.drivetrain.locked = False
        robot.io.set_voltage(0.0)
```

## Action Execution Flow

### Lifecycle

```
schedule_action() ->
Action added to scheduler ->
Each iteration (
  1. Action yields instruction
  2. Scheduler calls instruction.step()
  3. Scheduler checks instruction.is_complete()
  4. When complete, call instruction.on_complete()
  5. Resume action to next yield
) ->
Action returns (StopIteration) ->
Action removed from scheduler
```

### Cancellation Flow

```
cancel_action("name") ->
Instruction.cleanup() called ->
Action generator closed ->
Action removed from scheduler
```

### Error Handling

All exceptions are caught and converted to CRITICAL faults:
- Action execution errors - logged and cancelled
- Instruction.step() errors - logged and cancelled
- Cleanup errors - logged (action still removed)

---

## Common Patterns

### Sequential Actions

```python
def sequence(drivetrain: Drivetrain, intake: Intake) -> AsyncAction:
    yield from drivetrain.move_forward(3)
    yield from intake.spin_up()
    yield from wait(0.5)
    yield from intake.collect_pieces()
```

### Parallel Actions

```python
def timed_pickup(drivetrain: Drivetrain, intake: Intake) -> None:
    yield from parallel(
        drivetrain.move_forward(5),
        intake.collect_pieces()
    )
```

### Multi-Path Autonomous

```python
def adaptive_auto(robot: AdaptiveRobot) -> AsyncAction:
    game_state = robot.get_game_state()
    
    if game_state == States.LEFT:
        yield from left_auto_path(robot)
    elif game_state == States.RIGHT:
        yield from center_auto_path(robot)
    else:
        yield from right_auto_path(robot)
```

### Failsafe Autonomous

```python
def failsafe_auto(robot: AdaptiveRobot) -> None:
    yield from race(
        primary_autonomous_strategy(robot),
        wait(10.0)
    )

    yield from robot.drivetrain.move_forward(10)
```

### Conditional Loops

```python
def loop_until_complete(robot: AdaptiveRobot) -> None:
    attempts = 0
    max_attempts = 3

    while attempts < max_attempts:
        try:
            yield from robot.arm.move_to_target()
            break
        except Exception:
            attempts += 1
            if attempts < max_attempts:
                yield from wait(0.5) # wait before retry
```

### Parallel with Timeout

```python
def parallel_with_safety(robot: AdaptiveRobot) -> None:
    yield from with_timeout(
        parallel(
            robot.drivetrain.drive_path(),
            robot.intake.spin(),
            robot.arm.move_to_position()
        ),
        timeout=5.0
    )
```

---
