import logging
import pytest

from components.intake.intake import Intake
from components.intake.intake_io.testing_io import IntakeTestIO

from adaptive_robot.adaptive_component.adaptive_component import ComponentContext
from adaptive_robot.telemetry.telemetry import TelemetryPublisher
from adaptive_robot.telemetry.struct_telemetry import TelemetryStructPublisher


@pytest.fixture
def test_io() -> IntakeTestIO:
    """
    Creates a new test IO for each test.
    """
    return IntakeTestIO()


@pytest.fixture
def intake(test_io: IntakeTestIO) -> Intake:
    """
    Creates an intake component with test IO and context.
    """
    intake = Intake(io=test_io)

    telemetry = TelemetryPublisher()
    struct_telemetry = TelemetryStructPublisher()
    context = ComponentContext(
        telemetry=telemetry,
        struct_telemetry=struct_telemetry,
        logger=logging.getLogger("test_intake")
    )
    intake.context = context
    
    return intake


class TestIntakeSafeDefaults:
    """
    Tests that safe defaults are applied when necessary.
    """
    def test_safe_defaults_called_on_enabled(
        self,
        intake: Intake,
        test_io: IntakeTestIO
    ) -> None:
        """
        Tests if on_enabled forces zero voltage.
        """
        test_io.set_voltage(6.0)

        intake.on_enabled()

        voltage = test_io.get_last_commanded_voltage()
        assert voltage == 0.0
    
    def test_safe_defaults_called_on_disabled(
        self,
        intake: Intake,
        test_io: IntakeTestIO
    ) -> None:
        """
        Tests if on_disabled forces zero voltage.
        """
        test_io.set_voltage(6.0)
        
        intake.on_disabled()
        
        voltage = test_io.get_last_commanded_voltage()
        assert voltage == 0.0
    
    def test_safe_defaults_called_on_faulted_init(
        self,
        intake: Intake,
        test_io: IntakeTestIO
    ) -> None:
        """
        Tests if on_faulted_init forces zero voltage.
        """
        test_io.set_voltage(6.0)
        
        intake.on_faulted_init()
        
        voltage = test_io.get_last_commanded_voltage()
        assert voltage == 0.0
    
    def test_safe_defaults_called_on_faulted_periodic(
        self,
        intake: Intake,
        test_io: IntakeTestIO
    ) -> None:
        """
        Tests if on_faulted_periodic forces zero voltage.
        """
        test_io.set_voltage(6.0)
        
        intake.on_faulted_periodic()
        
        voltage = test_io.get_last_commanded_voltage()
        assert voltage == 0.0


class TestIntakeIntegration:
    """
    Simulates real scenarios for the intake.
    """
    def test_full_lifecycle(
        self,
        intake: Intake,
        test_io: IntakeTestIO
    ) -> None:
        """
        Simulate robot disabled, voltage commanded during on_enabled, and then re-disabled.
        """
        intake.on_disabled()
        voltage = test_io.get_last_commanded_voltage()
        assert voltage == 0.0

        intake.on_enabled()
        test_io.set_voltage(12.0)

        intake.on_disabled()
        voltage = test_io.get_last_commanded_voltage()
        assert voltage == 0.0, "Should zero on disable"
