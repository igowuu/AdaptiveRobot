from abc import ABC
from dataclasses import dataclass
from typing import Any, final

from adaptive_robot.telemetry.telemetry import TelemetryPublisher, primitive_type
from adaptive_robot.telemetry.struct_telemetry import TelemetryStructPublisher
from adaptive_robot.profiling.profile_method import profile_method


@dataclass
class TelemetryContext:
    """
    Holds necessary attributes for a TelemetryPublishable subclass to function properly.  
    These must be injected by the main scheduler loop to each instance.
    """
    telemetry: TelemetryPublisher
    struct_telemetry: TelemetryStructPublisher


class TelemetryPublishable(ABC):
    """
    Interface for objects that publish telemetry each iteration.
    """
    def __init_subclass__(cls) -> None:
        """
        Auto-initializes telemetry context attributes when a subclass is created.
        """
        super().__init_subclass__()
        cls._pending_telemetry_values: list[tuple[str, primitive_type]] = []
        cls._pending_struct_values: list[tuple[str, Any]] = []
        cls._telemetry_context: TelemetryContext | None = None

        # Manually decorate any `publish_telemetry()` implementations by default.
        if 'publish_telemetry' in cls.__dict__:
            original_publish_telemetry = cls.publish_telemetry
            cls.publish_telemetry = profile_method(original_publish_telemetry)

    @final
    @property
    def telemetry_context(self) -> TelemetryContext | None:
        """
        The user should not access this field directly.
        """
        return self._telemetry_context

    @final
    @telemetry_context.setter
    def telemetry_context(self, context: TelemetryContext) -> None:
        """ 
        All pending  objects declared in __init__ are
        automatically extended into the objects context.  
        """
        for value in self._pending_telemetry_values:
            context.telemetry.put_value(value[0], value[1])
        for value in self._pending_struct_values:
            context.struct_telemetry.put_struct_value(value[0], value[1])

        self._telemetry_context = context

    @final
    def publish_value(self, directory: str, value: primitive_type) -> None:
        """
        Publishes an immutable value to networktables.
        
        :param directory: The directory that the value will be saved under in NetworkTables.
        :param value: The value you wish to publish to NetworkTables.
        """
        if self._telemetry_context is None:
            self._pending_telemetry_values.append((directory, value))
        else:
            self._telemetry_context.telemetry.put_value(directory, value)
    
    @final
    def publish_struct_value(self, directory: str, value: object) -> None:
        """
        Publishes a WPIStruct (Pose3d, for example) to networktables.
        This is relatively performance heavy, so it should not be used excessively.

        :param directory: The directory that the value will be saved under in NetworkTables.
        :param value: The value you wish to publish to networktables. If not a WPIStruct, raises an error.
        """
        if self._telemetry_context is None:
            self._pending_struct_values.append((directory, value))
        else:
            self._telemetry_context.struct_telemetry.put_struct_value(directory, value)
    
    def publish_telemetry(self) -> None:
        """
        Called each iteration before scheduling, no matter the objects health.
        """
        ...
