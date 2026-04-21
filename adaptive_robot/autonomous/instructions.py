from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

from wpilib import Timer

if TYPE_CHECKING:
    from adaptive_robot.autonomous.async_actions import AsyncAction


class Instruction(ABC):
    """
    Represents an instruction that an async action can give 
    to the scheduler for custom behavior.
    """
    @abstractmethod
    def is_complete(self) -> bool:
        """
        Checks if this instruction is complete and the scheduler should resume the action.  
        """
        pass

    def step(self) -> None:
        """
        Executes one iteration of this instruction if needed.  
        """
        pass

    def on_complete(self) -> None:
        """
        Called when this instruction completes if needed.
        """
        pass

    def cleanup(self) -> None:
        """
        Clean up any resources used by this instruction if needed.
        """
        pass


class WaitInstruction(Instruction):
    """
    Instructs the scheduler to not call this action again 
    until the specified duration has elapsed.
    """
    def __init__(self, duration: float) -> None:
        self.duration = duration
        self.start_time = Timer.getFPGATimestamp()

    def is_complete(self) -> bool:
        """
        Completes the AsyncAction if the duration is met.
        """
        return Timer.getFPGATimestamp() - self.start_time >= self.duration

class TimeoutInstruction(Instruction):
    """
    Instruction that manages an action with a timeout.
    """
    def __init__(self, action: AsyncAction, timeout: float) -> None:
        self.action = action
        self.timeout = timeout
        self.start_time = Timer.getFPGATimestamp()
        self.current_instruction: Instruction | None = None
        self.completed = False
        self.timed_out = False
    
    def is_complete(self) -> bool:
        elapsed = Timer.getFPGATimestamp() - self.start_time
        
        if elapsed >= self.timeout:
            self.timed_out = True
            self.completed = True
            if self.current_instruction:
                self.current_instruction.cleanup()
            return True
        
        if self.current_instruction and self.current_instruction.is_complete():
            try:
                self.current_instruction = next(self.action)
            except StopIteration:
                self.completed = True
                return True
        
        return False
    
    def step(self) -> None:
        if self.current_instruction:
            self.current_instruction.step()
    
    def cleanup(self) -> None:
        if self.current_instruction:
            self.current_instruction.cleanup()
        try:
            self.action.close()
        except (StopIteration, GeneratorExit):
            pass


class RaceInstruction(Instruction):
    """
    Instruction that runs multiple actions and returns on first completion.
    """
    def __init__(self, *actions: AsyncAction) -> None:
        self.actions = list(actions)
        self.current_instructions: list[Instruction | None] = [None] * len(actions)
        self.completed_indices: set[int] = set()

        for i, action in enumerate(self.actions):
            try:
                self.current_instructions[i] = next(action)
            except StopIteration:
                self.completed_indices.add(i)
    
    def is_complete(self) -> bool:
        if len(self.completed_indices) > 0:
            return True
        
        for i, instruction in enumerate(self.current_instructions):
            if i not in self.completed_indices and instruction and instruction.is_complete():
                self.completed_indices.add(i)
                return True
        
        return False
    
    def step(self) -> None:
        for i, instruction in enumerate(self.current_instructions):
            if i not in self.completed_indices and instruction:
                instruction.step()
    
    def cleanup(self) -> None:
        for i, action in enumerate(self.actions):
            if self.current_instructions[i]:
                self.current_instructions[i].cleanup() # type: ignore[NoneAccountedFor]
            try:
                action.close()
            except (StopIteration, GeneratorExit):
                pass


class ParallelInstruction(Instruction):
    """
    Instruction that runs multiple actions concurrently until all complete.
    """
    def __init__(self, *actions: AsyncAction) -> None:
        self.actions = list(actions)
        self.current_instructions: list[Instruction | None] = [None] * len(actions)
        self.completed_indices: set[int] = set()

        for i, action in enumerate(self.actions):
            try:
                self.current_instructions[i] = next(action)
            except StopIteration:
                self.completed_indices.add(i)
    
    def is_complete(self) -> bool:
        return len(self.completed_indices) == len(self.actions)
    
    def step(self) -> None:
        for i, instruction in enumerate(self.current_instructions):
            if i not in self.completed_indices and instruction:
                instruction.step()
    
    def _advance_action(self, index: int) -> None:
        """
        Advances action at index to next instruction.
        """
        try:
            self.current_instructions[index] = next(self.actions[index])
        except StopIteration:
            self.completed_indices.add(index)
    
    def update(self) -> None:
        """
        Called by scheduler to advance completed instructions.
        """
        for i, instruction in enumerate(self.current_instructions):
            if i not in self.completed_indices and instruction and instruction.is_complete():
                self._advance_action(i)
