from typing import Generator, Any
from adaptive_robot.autonomous.instructions import (
    Instruction,
    WaitInstruction,
    TimeoutInstruction,
    ParallelInstruction,
    RaceInstruction
)


AsyncAction = Generator[Instruction | None, None, Any]


def wait(duration: float) -> AsyncAction:
    """
    Waits for a specified duration in seconds by yielding a WaitInstruction.
    
    :param duration: Time to wait in seconds.
    """
    if duration <= 0:
        return
    
    instruction = WaitInstruction(duration)
    while not instruction.is_complete():
        yield instruction


def with_timeout(action: AsyncAction, timeout: float) -> AsyncAction:
    """
    Runs an action with a timeout. Cancels if it exceeds duration.
    
    :param action: The generator-based action to run.
    :param timeout: Maximum duration in seconds.
    """
    instruction = TimeoutInstruction(action, timeout)
    try:
        instruction.current_instruction = next(action)
    except StopIteration:
        return
    
    while not instruction.is_complete():
        yield instruction


def race(*actions: AsyncAction) -> AsyncAction:
    """
    Runs multiple actions concurrently. Returns when first one finishes.
    
    :param actions: Variable number of generator-based actions to race.
    """
    if not actions:
        return
    
    instruction = RaceInstruction(*actions)
    while not instruction.is_complete():
        yield instruction


def parallel(*actions: AsyncAction) -> AsyncAction:
    """
    Runs multiple actions concurrently. Waits for all to finish.
    
    :param actions: Variable number of generator-based actions to run in parallel.
    """
    if not actions:
        return
    
    instruction = ParallelInstruction(*actions)
    
    while not instruction.is_complete():
        instruction.update()
        yield instruction
