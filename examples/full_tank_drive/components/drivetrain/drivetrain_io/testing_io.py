from wpimath.geometry import Rotation2d
from wpimath.units import volts, meters, meters_per_second

from components.drivetrain.drivetrain_io.io_base import DrivetrainIOBase
from components.drivetrain.drivetrain_constants import DriveConstants

from wpilib import Timer


class DrivetrainTestIO(DrivetrainIOBase):
    """
    Test IO that simulates actual hardware.
    """
    def __init__(self) -> None:
        self._left_voltage_commanded: volts = 0.0
        self._right_voltage_commanded: volts = 0.0

        self._left_velocity: meters_per_second = 0.0
        self._right_velocity: meters_per_second = 0.0
        self._left_distance: meters = 0.0
        self._right_distance: meters = 0.0
        self._angle: Rotation2d = Rotation2d()

        self._last_update_time = Timer.getFPGATimestamp()

        self._left_voltage_calls = 0
        self._right_voltage_calls = 0
        self._update_calls = 0

        self._battery_voltage: volts = 12.0
    
    def set_velocity_from_voltage(
        self,
        left_voltage: volts,
        right_voltage: volts
    ) -> None:
        """
        Sets the actual velocities based on commanded voltages.
        """
        self._left_velocity = (left_voltage / self._battery_voltage) * DriveConstants.MAX_LINEAR_SPEED
        self._right_velocity = (right_voltage / self._battery_voltage) * DriveConstants.MAX_LINEAR_SPEED
    
    def set_velocities(
        self,
        left_velocity: meters_per_second,
        right_velocity: meters_per_second
    ) -> None:
        """
        Directly sets the velocity readings.
        """
        self._left_velocity = left_velocity
        self._right_velocity = right_velocity
    
    def set_distances(
        self,
        left_distance: meters,
        right_distance: meters
    ) -> None:
        """
        Directly sets the distance readings.
        """
        self._left_distance = left_distance
        self._right_distance = right_distance
    
    def set_angle(self, angle: Rotation2d) -> None:
        """
        Directly sets the gyro angle.
        """
        self._angle = angle
    
    def get_left_velocity(self) -> meters_per_second:
        """
        Returns the simulated left velocity.
        """
        return self._left_velocity
    
    def get_right_velocity(self) -> meters_per_second:
        """
        Returns the simulated right velocity.
        """
        return self._right_velocity
    
    def get_left_distance(self) -> meters:
        """
        Returns the accumulated left distance.
        """
        return self._left_distance
    
    def get_right_distance(self) -> meters:
        """
        Returns the accumulated right distance.
        """
        return self._right_distance
    
    def get_left_voltage(self) -> volts:
        """
        Returns the commanded left voltage.
        """
        return self._left_voltage_commanded
    
    def get_right_voltage(self) -> volts:
        """
        Returns the commanded right voltage.
        """
        return self._right_voltage_commanded
    
    def get_angle(self) -> Rotation2d:
        """
        Returns the simulated gyro angle.
        """
        return self._angle

    def set_left_voltage(self, voltage: volts) -> None:
        """
        Commands voltage to left motors.
        Tracks call count for test assertions.
        """
        self._left_voltage_commanded = voltage
        self._left_voltage_calls += 1
    
    def set_right_voltage(self, voltage: volts) -> None:
        """
        Commands voltage to right motors.
        Tracks call count for test assertions.
        """
        self._right_voltage_commanded = voltage
        self._right_voltage_calls += 1

    def update(self) -> None:
        """
        Updates the simulation each iteration.

        Converts commanded voltages to velocities, converts velocities to positions, and tracks calls.
        """
        self._update_calls += 1

        self.set_velocity_from_voltage(
            self._left_voltage_commanded,
            self._right_voltage_commanded
        )

        now = Timer.getFPGATimestamp()
        dt = now - self._last_update_time
        
        if dt > 0 and dt < 1.0:
            self._left_distance += self._left_velocity * dt
            self._right_distance += self._right_velocity * dt

            angular_velocity = (self._right_velocity - self._left_velocity) / DriveConstants.TRACK_WIDTH
            angle_delta = angular_velocity * dt
            self._angle = Rotation2d(self._angle.radians() + angle_delta)
        
        self._last_update_time = now

    def get_left_voltage_call_count(self) -> int:
        """
        Returns number of times set_left_voltage was called.
        """
        return self._left_voltage_calls
    
    def get_right_voltage_call_count(self) -> int:
        """
        Returns number of times set_right_voltage was called.
        """
        return self._right_voltage_calls
    
    def get_update_call_count(self) -> int:
        """
        Returns number of times update was called.
        """
        return self._update_calls
    
    def reset_call_counts(self) -> None:
        """
        Resets call tracking.
        """
        self._left_voltage_calls = 0
        self._right_voltage_calls = 0
        self._update_calls = 0
    
    def get_last_commanded_voltages(self) -> tuple[volts, volts]:
        """
        Returns the most recently commanded voltages.
        """
        return (self._left_voltage_commanded, self._right_voltage_commanded)
    
    def assert_voltage_equal(
        self,
        expected_left: volts,
        expected_right: volts,
        tolerance: volts = 0.1
    ) -> None:
        """
        Asserts that the commanded voltages match expectations.
        """
        assert abs(self._left_voltage_commanded - expected_left) < tolerance, \
            f"Expected left voltage {expected_left} {tolerance}, got {self._left_voltage_commanded}V"
        assert abs(self._right_voltage_commanded - expected_right) < tolerance, \
            f"Expected right voltage {expected_right} {tolerance}, got {self._right_voltage_commanded}V"
