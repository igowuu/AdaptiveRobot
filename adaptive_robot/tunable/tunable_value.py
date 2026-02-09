
from typing import SupportsFloat, TypeVar, Generic

from ntcore import NetworkTableInstance


TunableType = TypeVar("TunableType", float, int, str, bool, list[SupportsFloat])


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
        self.default = default

        nt = NetworkTableInstance.getDefault()
        self.entry = nt.getEntry(directory)

        # Publish default
        if isinstance(default, bool):
            self.entry.setBoolean(default)
            self._getter = lambda: self.entry.getBoolean(default)
        elif isinstance(default, int):
            self.entry.setInteger(default)
            self._getter = lambda: self.entry.getInteger(default)
        elif isinstance(default, float):
            self.entry.setDouble(default)
            self._getter = lambda: self.entry.getDouble(default)
        elif isinstance(default, str):
            self.entry.setString(default)
            self._getter = lambda: self.entry.getString(default)
        else: # List of floats implied
            self.entry.setDoubleArray(default)
            self._getter = lambda: self.entry.getDoubleArray(default)

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
