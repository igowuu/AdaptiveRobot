from abc import ABC, abstractmethod
from typing import final

from adaptive_robot.interfaces.shared_interface_state import SharedHealthState
from adaptive_robot.profiling.profile_method import profile_method


class Schedulable(SharedHealthState, ABC):
    """
    Interface for objects that participate in robot scheduling.
    """
    FAULT_THRESHOLD = 10

    def __init_subclass__(cls) -> None:
        """
        Auto-initializes schedulable attributes when a subclass is created.
        All execute() methods are automatically profiled.
        """
        super().__init_subclass__()
        cls._fault_threshold = cls.FAULT_THRESHOLD
        cls._is_locked = False

        # Manually decorate any `execute()` implementations by default.
        if 'execute' in cls.__dict__:
            original_execute = cls.execute
            cls.execute = profile_method(original_execute)
    
    @final
    @property
    def locked(self) -> bool:
        """
        Returns True if the object is locked.
        """
        return self._is_locked

    @final
    @locked.setter
    def locked(self, locked: bool) -> None:
        """
        Locks an object, meaning that none of its hooks will be called (with the exception
        being publish_telemetry() if implemented from TelemetryPublishable).
        """
        self._is_locked = locked
    
    @final
    @property
    def fault_threshold(self) -> int:
        """
        Returns the fault threshold for the component.
        """
        return self._fault_threshold

    @final
    @fault_threshold.setter
    def fault_threshold(self, fault_threshold: int) -> None:
        """
        Sets the components fault threshold during runtime.
        """
        self._fault_threshold = fault_threshold

    def on_enabled(self) -> None:
        """
        Called once when robot transitions to an enabled mode.
        """
        ...
    
    def on_disabled(self) -> None:
        """
        Called once when robot transitions to a disabled mode.
        """
        ...
    
    def on_faulted_init(self) -> None:
        """
        Called once when the object first becomes unhealthy.
        """
        ...
    
    def on_faulted_periodic(self) -> None:
        """
        Called each iteration while the object is unhealthy.
        """
        ...

    @abstractmethod
    def execute(self) -> None:
        """
        Called each iteration while the object is healthy.
        """
        ...
