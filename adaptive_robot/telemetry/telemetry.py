from ntcore import NetworkTableInstance

from adaptive_robot.faults.faults import FaultSeverity
from adaptive_robot.interfaces.faultable import Faultable


primitive_type = bool | int | float | str


class TelemetryPublisher(Faultable):
    """
    TelementryPublisher creates a NetworkTable instance and allows values to be added
    to networktables through the put_value method. Values can be obtained through the 
    get_value method.
    """
    def __init__(self, rounding_digits: int = 5) -> None:
        self.nt = NetworkTableInstance.getDefault().getTable("Dashboard")
        self._last_values: dict[str, primitive_type] = {}

        self._default_round_digits = rounding_digits
        self._per_key_rounding: dict[str, int] = {}
    
    def set_rounding(self, key: str, decimal_places: int) -> None:
        """
        Sets the number of decimal places to round a specific telemetry key to.
        
        :param key: The telemetry key to configure.
        :param decimal_places: The number of decimal places to round to (0 or positive).

        :raises FaultException: If decimal_places is negative.
        """
        if decimal_places < 0:
            message = f"Decimal places must be non-negative, got {decimal_places}"
            self._raise_fault(None, FaultSeverity.ERROR, message, None)
            return

        self._per_key_rounding[key] = decimal_places
    
    def _get_rounding_digits(self, key: str) -> int:
        """
        Gets the number of decimal places to use for a given key.
        Returns per-key setting if configured, otherwise default.
        """
        return self._per_key_rounding.get(key, self._default_round_digits)
    
    def _publish(self, key: str, value: primitive_type) -> None:
        """
        Infers the type of a value and publishes it to the NT entry with the given key.
        
        :raises FaultException: Upon an unexpected error.
        """
        if isinstance(value, float):
            value = self._round(key, value)

        try:
            if isinstance(value, bool):
                self.nt.putBoolean(key, value)
            elif isinstance(value, str):
                self.nt.putString(key, value)
            else:
                self.nt.putNumber(key, value)

        except Exception as e:
            message = f"NT publish failed for key='{key}', value='{value}': {e}"
            self._raise_fault(None, FaultSeverity.ERROR, message, e)
    
    def _round(self, key: str, value: float) -> float:
        """
        Rounds a float value using the configured precision for the given key.
        """
        digits = self._get_rounding_digits(key)
        return round(value, digits)
    
    def put_value(self, key: str, value: primitive_type) -> None:
        """
        Puts a value to a NT instance if it has changed.
        If it has not changed, returns without doing anything (the value is the same).
        Float values are rounded according to the configured precision for this key.

        :raises FaultException: Upon an unexpected error.
        """
        if isinstance(value, float):
            value = self._round(key, value)

        if key in self._last_values:
            cached = self._last_values[key]
            if type(cached) == type(value) and cached == value:
                return

        self._last_values[key] = value
        self._publish(key, value)

    def get_value(self, key: str) -> primitive_type:
        """
        Returns the value found at the specified key.
        """
        entry = self.nt.getEntry(key)
        value = entry.getValue()

        if value.isDouble():
            raw_value = self._round(key, value.value())
        else:
            raw_value = value.value()

        self._last_values[key] = raw_value
        return raw_value
