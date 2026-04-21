
from typing import TypeVar, Generic

from ntcore import NetworkTableInstance


TunableType = TypeVar("TunableType", float, int, str, bool)


class TunableValue(Generic[TunableType]):
    """
    TunableValue creates a value that can be changed through NetworkTables and will update
    in the codebase. 

    TunableValue does not change any values (including objects) previously made with the value on update.
    """
    def __init__(self, directory: str, default: TunableType) -> None:
        """
        Creates an object that can be altered through NetworkTables.
        
        :param directory: The directory that the value will be saved under in NetworkTables
        :param default: The default value published at runtime
        """
        self._default = default

        nt = NetworkTableInstance.getDefault()
        self._entry = nt.getEntry(directory)

        # Publish default
        if isinstance(default, bool):
            self._entry.setBoolean(default)
            self._getter = lambda: self._entry.getBoolean(default)
        elif isinstance(default, int):
            self._entry.setInteger(default)
            self._getter = lambda: self._entry.getInteger(default)
        elif isinstance(default, float):
            self._entry.setDouble(default)
            self._getter = lambda: self._entry.getDouble(default)
        else: # String implied
            self._entry.setString(default)
            self._getter = lambda: self._entry.getString(default)

        self._last_value = self._getter()

    @property
    def value(self) -> TunableType:
        """
        Returns the last recorded value of the tunable object.
        """
        return self._last_value

    def update(self) -> bool:
        """
        Updates the internal value and returns True if the value has been changed since last checked.
        """
        current = self._getter()
        if current != self._last_value:
            self._last_value = current
            return True
        return False
