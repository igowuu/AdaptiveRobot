from wpimath.units import volts, radians, radians_per_second

from components.arm.arm_io.io_base import ArmIOBase
from components.arm.arm_constants import ArmConstants

from wpilib import Timer


class ArmTestIO(ArmIOBase):
    """
    Test IO that simulates actual hardware.
    """
    def __init__(self) -> None:
        self._voltage_commanded: volts = 0.0

        self._velocity: radians_per_second = 0.0
        self._position: radians = 0.0

        self._last_update_time = Timer.getFPGATimestamp()

        self._voltage_calls = 0
        self._update_calls = 0

        self._battery_voltage: volts = 12.0
    
    def set_velocity_from_voltage(
        self,
        voltage: volts
    ) -> None:
        """
        Sets the actual velocity based on commanded voltage.
        """
        self._velocity = (voltage / self._battery_voltage) * ArmConstants.MAX_VELOCITY
    
    def set_velocity(
        self,
        velocity: radians_per_second
    ) -> None:
        """
        Directly sets the velocity reading.
        """
        self._velocity = velocity
    
    def set_position(
        self,
        angle: radians
    ) -> None:
        """
        Directly sets the position reading.
        """
        self._position = angle
    
    def get_velocity(self) -> radians_per_second:
        """
        Returns the simulated velocity.
        """
        return self._velocity
    
    def get_position(self) -> radians:
        """
        Returns the current arm position.
        """
        return self._position
    
    def get_voltage(self) -> volts:
        """
        Returns the commanded voltage.
        """
        return self._voltage_commanded

    def set_voltage(self, voltage: volts) -> None:
        """
        Commands voltage to arm motors.
        Tracks call count for test assertions.
        """
        self._voltage_commanded = voltage
        self._voltage_calls += 1

    def update(self) -> None:
        """
        Updates the simulation each iteration.

        Converts commanded voltages to velocities, converts velocities to positions, and tracks calls.
        """
        self._update_calls += 1

        self.set_velocity_from_voltage(self._voltage_commanded)

        now = Timer.getFPGATimestamp()
        dt = now - self._last_update_time
        
        if dt > 0 and dt < 1.0:
            self._position += self._velocity * dt
        
        self._last_update_time = now

    def get_voltage_call_count(self) -> int:
        """
        Returns number of times set_voltage was called.
        """
        return self._voltage_calls
    
    def get_update_call_count(self) -> int:
        """
        Returns number of times update was called.
        """
        return self._update_calls
    
    def reset_call_counts(self) -> None:
        """
        Resets call tracking.
        """
        self._voltage_calls = 0
        self._update_calls = 0
    
    def get_last_commanded_voltage(self) -> volts:
        """
        Returns the most recently commanded voltage.
        """
        return self._voltage_commanded
    
    def assert_voltage_equal(
        self,
        expected: volts,
        tolerance: volts = 0.1
    ) -> None:
        """
        Asserts that the commanded voltage matches expectations.
        """
        assert abs(self._voltage_commanded - expected) < tolerance, \
            f"Expected voltage {expected} {tolerance}, got {self._voltage_commanded}"
