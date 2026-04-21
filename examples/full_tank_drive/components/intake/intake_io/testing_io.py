from wpimath.units import volts

from components.intake.intake_io.io_base import IntakeIOBase


class IntakeTestIO(IntakeIOBase):
    """
    Test IO that simulates actual hardware.
    """
    def __init__(self) -> None:
        self._voltage_commanded: volts = 0.0

        self._voltage_calls = 0
    
    def get_voltage(self) -> volts:
        return self._voltage_commanded

    def get_last_commanded_voltage(self) -> volts:
        """
        Returns the most recently commanded voltage.
        """
        return self._voltage_commanded
    
    def get_voltage_call_count(self) -> int:
        """
        Returns number of times set_voltage was called.
        """
        return self._voltage_calls
    
    def reset_call_counts(self) -> None:
        """
        Resets call tracking.
        """
        self._voltage_calls = 0

    def set_voltage(self, voltage: volts) -> None:
        """
        Commands voltage to intake motor.
        Tracks call count for test assertions.
        """
        self._voltage_commanded = voltage
        self._voltage_calls += 1

    def update(self) -> None:
        """
        Updates the simulation each iteration.
        """
        pass

    def assert_voltage_equal(
        self,
        expected: volts,
        tolerance: volts = 0.1
    ) -> None:
        """
        Asserts that the commanded voltage matches expectations.
        """
        assert abs(self._voltage_commanded - expected) < tolerance, \
            f"Expected voltage {expected} ±{tolerance}, got {self._voltage_commanded}V"
