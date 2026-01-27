from typing import final, Any
from abc import abstractmethod, ABC

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.tunable import TunableValue, TunableType
from adaptive_robot.telemetry import primitive_type

class AdaptiveComponent(ABC):
    """
    All hardware writes must originate from exactly one component per loop.
    All other components may only publish intent.

    tunable(name: str, default: float, table: str) - Creates a tunable constant that can be tuned through
    networktables. This does not update objects previously created with the constants. 
    """
    def __init__(self, robot: "AdaptiveRobot") -> None:
        self._robot = robot
        self._robot.add_component(self)

        self._tunables: list[TunableValue[Any]] = []

    @final
    def update_tunable_constants(self) -> None:
        for tunable in self._tunables:
            tunable.update()

    @final
    def tunable(self, key: str, default: TunableType) -> TunableValue[Any]:
        """
        Creates an object that can be altered through NetworkTables and will update the variable
        in your codebase.

        TunableValue does not change any values previously made with the value. If you wish to
        update associated objects as well, you will have to do it manually.
        
        :param key: The directory that the value will be saved under in NetworkTables
        :param default: The default value published at runtime
        """
        value = TunableValue(key, default)
        self._tunables.append(value)
        return value
    
    @final
    def publish_value(self, key: str, value: primitive_type) -> None:
        """
        Publishes a immutable value to networktables (display only).
        
        :param key: The directory that the value will be saved under in NetworkTables
        :param value: The value you wish to save publish to NetworkTables
        """
        self._robot.telemetry_publisher.put_value(key, value)

    def publish_telemetry(self) -> None:
        """
        Method automatically called every loop in the program before execute().
        """
        ...
    
    @abstractmethod
    def execute(self) -> None: 
        """
        Method automatically called every loop in the program.
        """
        ...