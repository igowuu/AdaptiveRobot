
import atexit
from pathlib import Path
from typing import Any, Generic, TypeVar

from ntcore import NetworkTableInstance

from adaptive_robot.utils.json_io import get_json_data, log_json_data


TunableType = TypeVar("TunableType", float, int, str, bool)


class TunablePersistenceManager:
    """
    Singleton that manages loading and saving tunable values to disk.
    """
    _file_path = Path(__file__).resolve().parents[2] / "tunables.json"
    _loaded = False
    _cache: dict[str, Any] = {}
    _registry: dict[str, "TunableValue[Any]"] = {}

    @classmethod
    def _ensure_loaded(cls) -> None:
        """
        Retrieves/loads JSON data if not already loaded.
        """
        if cls._loaded:
            return

        cls._loaded = True
        try:
            raw_data = get_json_data(cls._file_path, default={})
            if isinstance(raw_data, dict):
                cls._cache = raw_data
        except Exception:
            cls._cache = {}

    @classmethod
    def get_value(cls, key: str) -> Any | None:
        """
        Returns a value given a key. Returns None if not found.
        """
        cls._ensure_loaded()
        return cls._cache.get(key)

    @classmethod
    def register(cls, tunable: "TunableValue[Any]") -> None:
        """
        Registers a TunableValue instance into TunablePersistanceManager.
        """
        cls._ensure_loaded()
        cls._registry[tunable._directory] = tunable # type: ignore[AllowedProtectedUsage]

    @classmethod
    def save_all(cls) -> None:
        """
        Saves all tunables to a JSON.
        """
        cls._ensure_loaded()

        persisted = dict(cls._cache)
        for directory, tunable in cls._registry.items():
            persisted[directory] = tunable._getter()    # type: ignore[AllowedProtectedUsage]

        try:
            log_json_data(cls._file_path, persisted)
            cls._cache = persisted
        except Exception:
            pass


atexit.register(TunablePersistenceManager.save_all)


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
        persisted_value = TunablePersistenceManager.get_value(directory)
        if persisted_value is not None:
            self._default = persisted_value
        else:
            self._default = default

        self._directory = directory

        nt = NetworkTableInstance.getDefault()
        self._entry = nt.getEntry(directory)

        # Publish default or restored value
        if isinstance(self._default, bool):
            self._entry.setBoolean(self._default)
            self._getter = lambda: self._entry.getBoolean(self._default)
        elif isinstance(self._default, int):
            self._entry.setInteger(self._default)
            self._getter = lambda: self._entry.getInteger(self._default)
        elif isinstance(self._default, float):
            self._entry.setDouble(self._default)
            self._getter = lambda: self._entry.getDouble(self._default)
        else:  # String implied
            self._entry.setString(self._default)
            self._getter = lambda: self._entry.getString(self._default)

        self._last_value = self._getter()
        TunablePersistenceManager.register(self)

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
