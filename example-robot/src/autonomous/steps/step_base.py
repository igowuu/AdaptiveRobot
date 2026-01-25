from abc import ABC, abstractmethod

class Step(ABC):
    @abstractmethod
    def start(self) -> None:
        """Called once when step begins."""
        pass

    @abstractmethod
    def update(self) -> None:
        """Called every robot loop."""
        pass

    @abstractmethod
    def is_finished(self) -> bool:
        """Return True when step is complete."""
        pass

    @abstractmethod
    def stop(self) -> None:
        """Stop this step immediately."""
        pass
