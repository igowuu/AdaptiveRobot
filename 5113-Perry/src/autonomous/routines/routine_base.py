from abc import ABC, abstractmethod
from collections.abc import Sequence

from autonomous.steps.step_base import Step


class RoutineBase(ABC):
    """
    Abstract base class for all autonomous routines.
    Defines the interface: start, update, is_finished, stop.
    """

    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def update(self) -> None: ...

    @abstractmethod
    def is_finished(self) -> bool: ...

    @abstractmethod
    def stop(self) -> None: ...

class SequentialRoutine(RoutineBase):
    """
    Class to define a sequential routine, where each step 
    waits for the previous to end before executing.
    """

    def __init__(self, steps: Sequence[Step]) -> None:
        self.steps = steps
        self.current_index = 0

    def start(self) -> None:
        self.current_index = 0
        if self.steps:
            self.steps[0].start()

    def update(self) -> None:
        if self.current_index >= len(self.steps):
            return

        current = self.steps[self.current_index]
        current.update()

        if current.is_finished():
            current.stop()
            self.current_index += 1
            if self.current_index < len(self.steps):
                self.steps[self.current_index].start()

    def is_finished(self) -> bool:
        return self.current_index >= len(self.steps)

    def stop(self) -> None:
        if self.current_index < len(self.steps):
            print("but the cond?")
            self.steps[self.current_index].stop()
