from typing import Optional

import wpilib

from ntcore import NetworkTableInstance


primitive_type = bool | int | float | str


class TelemetryPublisher:
    """
    TelementryPublisher creates a NetworkTable instance and allows values to be added
    to networktables through the put_value method. Values can be obtained through the 
    get_value method.
    """
    def __init__(self) -> None:
        self.nt = NetworkTableInstance.getDefault().getTable("Dashboard")
        self._last_values: dict[str, primitive_type] = {}

        self._round_digits = 5
    
    def _publish(self, key: str, value: primitive_type) -> None:
        try:
            if isinstance(value, bool):
                self.nt.putBoolean(key, value)
            elif isinstance(value, str):
                self.nt.putString(key, value)
            else:
                self.nt.putNumber(key, value)

        except Exception as e:
            wpilib.reportError(
                error=f"NT publish failed for key='{key}', value='{value}': {e}",
                printTrace=False
            )
    
    def _round(self, value: float) -> float:
        return round(value, self._round_digits)
    
    def put_value(self, key: str, value: primitive_type) -> None:
        """
        Publishes a value to NetworkTables under the given key. Keys can specify folders
        via foward slashes. For example, a key within the Drivetrain folder could be Drivetrain/FLMotor.
        """
        # Only update networktables if values change
        if key in self._last_values and self._last_values[key] == value:
            return
        
        # Round floats and ints to the nearest _round_digits
        if isinstance(value, float):
            value = self._round(value)

        self._last_values[key] = value
        self._publish(key, value)

    def get_value(self, key: str) -> Optional[primitive_type]:
        """
        Returns the value found at the specified key. If not found, returns None.
        """
        value_found = self._last_values.get(key)

        if value_found is None:
            return None
        return value_found
