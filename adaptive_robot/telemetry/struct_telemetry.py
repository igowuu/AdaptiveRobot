from dataclasses import dataclass
from typing import Any, Optional

from ntcore import NetworkTableInstance, StructPublisher, StructSubscriber

from adaptive_robot.interfaces.faultable import Faultable
from adaptive_robot.faults.faults import FaultSeverity


@dataclass
class StructEntry:
    """
    Holds a struct publisher, subscriber, and its cached value for change detection.
    """
    publisher: StructPublisher
    subscriber: StructSubscriber
    cached_value: Any


class TelemetryStructPublisher(Faultable):
    """
    Publishes WPILib struct types (Pose3d, Pose2d, etc.) through NetworkTables.  
    Uses caching to significantly help with excess process speed.  
    This is relatively performance heavy, so it should not be used excessively.
    """
    def __init__(self) -> None:
        self._entries: dict[str, StructEntry] = {}

    def put_struct_value(self, directory: str, value: object) -> None:
        """
        Publishes a WPILib struct-compatible value to NetworkTables.

        :param directory: NetworkTables entry key (path). Use forward slashes for folders.
        :param value: The struct instance to publish (i.e. Pose3d(3, 4, 5)).

        :raises FaultException: Upon an unexpected error.
        """
        try:
            if directory not in self._entries:
                value_type = type(value)
                publisher = NetworkTableInstance.getDefault() \
                    .getStructTopic("Dashboard/" + directory, value_type) \
                    .publish()

                subscriber = publisher.getTopic().subscribe(value_type())
                self._entries[directory] = StructEntry(publisher, subscriber, value)

                publisher.set(value)
            else:
                entry = self._entries[directory]
                if entry.cached_value != value:
                    entry.publisher.set(value)
                    entry.cached_value = value

        except Exception as e:
            message = f"Struct publish failed for directory= {directory}: {e}"
            self.raise_fault(None, FaultSeverity.ERROR, message, e)

    def get_struct_value(self, key: str) -> Optional[Any]:
        """
        Returns the value in networktables for a given struct key.
        Returns None if the key does not exist.

        :raises FaultException: Upon an unexpected error.
        """
        try:
            if key not in self._entries:
                return None
            return self._entries[key].subscriber.get()
        except Exception as e:
            message = f"Failed to get struct value for key='{key}': {e}"
            self.raise_fault(None, FaultSeverity.ERROR, message, e)
