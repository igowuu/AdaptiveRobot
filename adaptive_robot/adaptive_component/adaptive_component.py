from typing import final, Any
from abc import abstractmethod, ABC
from dataclasses import dataclass, field
import logging

from adaptive_robot.tunable.tunable_value import TunableValue, TunableType
from adaptive_robot.tunable.tunable_pid_controller import TunablePIDController
from adaptive_robot.telemetry.telemetry import TelemetryPublisher, primitive_type
from adaptive_robot.telemetry.struct_telemetry import TelemetryStructPublisher
from adaptive_robot.interfaces.faultable import Faultable


logger = logging.getLogger(__name__)


@dataclass
class ComponentContext:
    """
    Holds necessary attributes for a component to function properly.  
    These must be injected by the main scheduler loop to each AdaptiveComponent instance.
    """
    telemetry: TelemetryPublisher
    struct_telemetry: TelemetryStructPublisher
    logger: logging.Logger
    tunables: list[TunableValue[Any]] = field(default_factory=list[TunableValue[Any]])
    tunable_pids: list[TunablePIDController] = field(default_factory=list[TunablePIDController])


class AdaptiveComponent(Faultable, ABC):
    """
    This base class defines the execution and telemetry lifecycle for all robot components.
    Subclasses should implement execute() and may override publish_telemetry().
    """
    def __init__(self, context: ComponentContext | None = None) -> None:
        """
        Creates an AdaptiveComponent object to register into the scheduler.

        :param context: Optional ComponentContext containing telemetry and logger references.
        """
        self._is_healthy = True
        self._is_locked = False
        self._context = context
        self._pending_tunables: list[TunableValue[Any]] = []
        self._pending_tunable_pids: list[TunablePIDController] = []
        self._pending_telemetry_values: list[tuple[str, primitive_type]] = []
        self._pending_struct_values: list[tuple[str, Any]] = []

    @final
    @property
    def context(self) -> ComponentContext | None:
        """
        The user should not access this field directly.
        Used by the component subscheduler to get the component context.  
        """
        return self._context

    @final
    @context.setter
    def context(self, context: ComponentContext) -> None:
        """
        The user should not change this field directly.  
        Used by the component subscheduler to set the component context.  
        All pending tunable and tunable pid objects declared in __init__ are
        automatically extended into the components context.  
        """
        context.tunables.extend(self._pending_tunables)
        context.tunable_pids.extend(self._pending_tunable_pids)

        for value in self._pending_telemetry_values:
            context.telemetry.put_value(value[0], value[1])
        for value in self._pending_struct_values:
            context.struct_telemetry.put_struct_value(value[0], value[1])

        self._context = context

    @final
    def set_health(self, is_healthy: bool) -> None:
        """
        Used by the component subscheduler to make a component healthy or unhealthy.  
        The user should not call this method.
        """
        self._is_healthy = is_healthy

    @final
    def is_healthy(self) -> bool:
        """
        Returns True if no faults within the component have been detected.
        """
        return self._is_healthy

    @final
    @property
    def locked(self) -> bool:
        """
        Returns True if the component is locked.
        """
        return self._is_locked

    @final
    @locked.setter
    def locked(self, locked: bool) -> None:
        """
        Locks a component, meaning that none of its hooks will be called (with the exception
        being publish_telemetry()).
        """
        self._is_locked = locked

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
        if self._context is None:
            self._pending_tunables.append(value)
        else:
            self._context.tunables.append(value)

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
        if self._context is None:
            self._pending_tunable_pids.append(tunablePID)
        else:
            self._context.tunable_pids.append(tunablePID)

        return tunablePID
    
    @final
    def publish_value(self, directory: str, value: primitive_type) -> None:
        """
        Publishes an immutable value to networktables.
        
        :param directory: The directory that the value will be saved under in NetworkTables.
        :param value: The value you wish to publish to NetworkTables.
        """
        if self._context is None:
            self._pending_telemetry_values.append((directory, value))
        else:
            self._context.telemetry.put_value(directory, value)
    
    @final
    def publish_struct_value(self, directory: str, value: object) -> None:
        """
        Publishes a WPIStruct (Pose3d, Field3d, etc) to networktables.
        This is relatively performance heavy, so it should not be used excessively.

        :param directory: The directory that the value will be saved under in NetworkTables.
        :param value: The value you wish to publish to networktables. If not a WPIStruct, raises an error.
        """
        if self._context is None:
            self._pending_struct_values.append((directory, value))
        else:
            self._context.struct_telemetry.put_struct_value(directory, value)

    def on_enabled(self) -> None:
        """
        Method that is called once when the robot is enabled.
        """
        ...

    def on_disabled(self) -> None:
        """
        Method that is called once when the robot is disabled.
        """
        ...

    def on_faulted_init(self) -> None:
        """
        Method that is called once when the component first becomes unhealthy.

        The user should implement logic here to ensure that hardware defaults to safe values.
        """
        ...

    def on_faulted_periodic(self) -> None:
        """
        Method that is called each iteration if the component is no longer healthy.

        The user should implement logic here to ensure that hardware defaults to safe values.
        """
        ...

    def publish_telemetry(self) -> None:
        """
        Method automatically called every loop in the program before execute().  

        Telemetry values that need to be published each iteration should go here.
        """
        ...

    @abstractmethod
    def execute(self) -> None: 
        """
        Method automatically called every loop in the program.

        Hardware should be commanded in this method.
        """
        ...
