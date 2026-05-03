from abc import ABC
from typing import Any, final
from dataclasses import dataclass, field

from adaptive_robot.tunable.tunable_value import TunableValue, TunableType
from adaptive_robot.tunable.tunable_pid_controller import TunablePIDController


@dataclass
class TunableContext:
    """
    Holds necessary attributes for a TunablePublishable subclass to function properly.  
    These must be injected by the main scheduler loop to each instance.
    """
    tunables: list[TunableValue[Any]] = field(default_factory=list[TunableValue[Any]])
    tunable_pids: list[TunablePIDController] = field(default_factory=list[TunablePIDController])


class TunablePublishable(ABC):
    """
    Interface for objects that use tunable methods.
    """
    def __init_subclass__(cls) -> None:
        """
        Auto-initializes tunable context attributes when a subclass is created.
        """
        super().__init_subclass__()
        cls._pending_tunables: list[TunableValue[Any]] = []
        cls._pending_tunable_pids: list[TunablePIDController] = []
        cls._tunable_context: TunableContext | None = None

    @final
    @property
    def tunable_context(self) -> TunableContext | None:
        """
        The user should not access this field directly.
        """
        return self._tunable_context

    @final
    @tunable_context.setter
    def tunable_context(self, context: TunableContext) -> None:
        """ 
        All pending tunable and tunable pid objects declared in __init__ are
        automatically extended into the objects context.  
        """
        context.tunables.extend(self._pending_tunables)
        context.tunable_pids.extend(self._pending_tunable_pids)

        self._tunable_context = context


    @final
    def tunable(self, directory: str, default: TunableType) -> TunableValue[Any]:
        """
        Creates an object that can be altered through NetworkTables and will update the variable
        in the codebase.

        TunableValue does not inherently change any values (including objects) previously 
        made with the value.
        
        :param directory: The directory that the value will be saved under in NetworkTables.
        :param default: The default value published at runtime.
        """
        value = TunableValue(directory, default)
        if self._tunable_context is None:
            self._pending_tunables.append(value)
        else:
            self._tunable_context.tunables.append(value)

        return value
    
    @final
    def tunablePID(
        self,
        kp: float,
        ki: float,
        kd: float,
        directory: str = "Tunables/PIDController",
        period: float = 0.02
    ) -> TunablePIDController:
        """
        Creates an object that can be altered through NetworkTables and will update a TunablePIDController
        object every iteration in the codebase.
        
        :param kp: The proportional coefficient. Should be >= 0.
        :param ki: The integral coefficient. Should be >= 0.
        :param kd: The derivative coefficient. Should be >= 0.
        :param period: The period between controller updates in seconds. The default is 20 milliseconds. Must be positive.
        :param directory: The directory in NetworkTables in which the tunable PID values will be located at.
        """
        tunablePID = TunablePIDController(
            kp=TunableValue(f"{directory}/kp", kp),
            ki=TunableValue(f"{directory}/ki", ki), 
            kd=TunableValue(f"{directory}/kd", kd),
            period=period
        )
        if self._tunable_context is None:
            self._pending_tunable_pids.append(tunablePID)
        else:
            self._tunable_context.tunable_pids.append(tunablePID)

        return tunablePID
