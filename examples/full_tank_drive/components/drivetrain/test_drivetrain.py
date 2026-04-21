import logging
import pytest

from components.drivetrain.drivetrain import Drivetrain, DriveMode
from components.drivetrain.drivetrain_io.testing_io import DrivetrainTestIO

from adaptive_robot import BasicPriority
from adaptive_robot.adaptive_component.adaptive_component import ComponentContext
from adaptive_robot.telemetry.telemetry import TelemetryPublisher
from adaptive_robot.telemetry.struct_telemetry import TelemetryStructPublisher


@pytest.fixture
def test_io() -> DrivetrainTestIO:
    """
    Creates a new test IO for each test.
    """
    return DrivetrainTestIO()


@pytest.fixture
def drivetrain(test_io: DrivetrainTestIO) -> Drivetrain:
    """
    Creates a drivetrain component with test IO and context.
    """
    drivetrain = Drivetrain(io=test_io)

    telemetry = TelemetryPublisher()
    struct_telemetry = TelemetryStructPublisher()
    context = ComponentContext(
        telemetry=telemetry,
        struct_telemetry=struct_telemetry,
        logger=logging.getLogger("test_drivetrain")
    )
    drivetrain.context = context
    
    return drivetrain


class TestDrivetrainRequestArbitration:
    """
    Tests for request arbitration with AxisController.
    """
    def test_single_request_resolution(
        self,
        drivetrain: Drivetrain
    ) -> None:
        """
        Tests that a single request per axis correctly gets resolved.
        """
        drivetrain.request_linear_velocity(1.0, BasicPriority.TELEOP, "teleop")
        drivetrain.request_angular_velocity(0.0, BasicPriority.TELEOP, "teleop")

        linear_resolved = drivetrain.linear_velocity_controller.resolve()
        angular_resolved = drivetrain.angular_velocity_controller.resolve()
        
        assert linear_resolved.value == 1.0
        assert linear_resolved.source == "teleop"
        assert linear_resolved.priority == BasicPriority.TELEOP.value
        assert angular_resolved.value == 0.0
    
    def test_higher_priority_override(
        self,
        drivetrain: Drivetrain
    ) -> None:
        """
        Test priority ordering with BasicPriority (safety > auto > teleop).
        """
        drivetrain.request_linear_velocity(1.0, BasicPriority.TELEOP, "teleop")
        drivetrain.request_linear_velocity(0.5, BasicPriority.AUTO, "auto")
        
        resolved = drivetrain.linear_velocity_controller.resolve()
        assert resolved.value == 0.5
        assert resolved.source == "auto"

        drivetrain.request_linear_velocity(0.0, BasicPriority.SAFETY, "safety")
        
        resolved = drivetrain.linear_velocity_controller.resolve()
        assert resolved.value == 0.0
        assert resolved.source == "safety"
    
    def test_most_recent_wins(
        self,
        drivetrain: Drivetrain
    ) -> None:
        """
        When same priority, most recent request should win.
        """
        drivetrain.request_linear_velocity(1.0, BasicPriority.TELEOP, "teleop_1")
        drivetrain.request_linear_velocity(0.5, BasicPriority.TELEOP, "teleop_2")
        
        resolved = drivetrain.linear_velocity_controller.resolve()
        assert resolved.value == 0.5


class TestDrivetrainSafeDefaults:
    """
    Tests that safe defaults are applied when necessary.
    """
    def test_safe_defaults_called_on_enabled(
        self,
        drivetrain: Drivetrain,
        test_io: DrivetrainTestIO
    ) -> None:
        """
        Tests if on_enabled forces zero velocity.
        """
        test_io.set_left_voltage(6.0)
        test_io.set_right_voltage(6.0)

        drivetrain.on_enabled()

        left_v, right_v = test_io.get_last_commanded_voltages()
        assert left_v == 0.0
        assert right_v == 0.0
    
    def test_safe_defaults_called_on_disabled(
        self,
        drivetrain: Drivetrain,
        test_io: DrivetrainTestIO
    ) -> None:
        """
        Tests if on_disabled forces zero velocity.
        """
        test_io.set_left_voltage(6.0)
        test_io.set_right_voltage(6.0)
        
        drivetrain.on_disabled()
        
        left_v, right_v = test_io.get_last_commanded_voltages()
        assert left_v == 0.0
        assert right_v == 0.0
    
    def test_safe_defaults_called_on_faulted_init(
        self,
        drivetrain: Drivetrain,
        test_io: DrivetrainTestIO
    ) -> None:
        """
        Tests if on_faulted_init forces zero velocity.
        """
        test_io.set_left_voltage(6.0)
        test_io.set_right_voltage(6.0)
        
        drivetrain.on_faulted_init()
        
        left_v, right_v = test_io.get_last_commanded_voltages()
        assert left_v == 0.0
        assert right_v == 0.0
    
    def test_safe_defaults_called_on_faulted_periodic(
        self,
        drivetrain: Drivetrain,
        test_io: DrivetrainTestIO
    ) -> None:
        """
        Tests if on_faulted_periodic forces zero velocity.
        """
        test_io.set_left_voltage(6.0)
        test_io.set_right_voltage(6.0)
        
        drivetrain.on_faulted_periodic()
        
        left_v, right_v = test_io.get_last_commanded_voltages()
        assert left_v == 0.0
        assert right_v == 0.0


class TestDrivetrainModes:
    """
    Tests for drive mode switching.
    """
    def test_default_closed_loop(
        self,
        drivetrain: Drivetrain
    ) -> None:
        """
        By default, drivetrain should use closed-loop control.
        """
        assert drivetrain.drive_mode == DriveMode.CLOSED_LOOP
    
    def test_switch_drive_mode(
        self,
        drivetrain: Drivetrain
    ) -> None:
        """
        Tests switching between modes.
        """
        drivetrain.drive_mode = DriveMode.OPEN_LOOP
        assert drivetrain.drive_mode == DriveMode.OPEN_LOOP
        
        drivetrain.drive_mode = DriveMode.CLOSED_LOOP
        assert drivetrain.drive_mode == DriveMode.CLOSED_LOOP


class TestDrivetrainIntegration:
    """
    Simulates real scenarios for the drivetrain.
    """
    def test_full_lifecycle(
        self,
        drivetrain: Drivetrain,
        test_io: DrivetrainTestIO
    ) -> None:
        """
        Simulate robot disabled, request made during on_enabled, and then re-disabled.
        """
        drivetrain.on_disabled()
        left_v, right_v = test_io.get_last_commanded_voltages()
        assert left_v == 0.0 and right_v == 0.0

        drivetrain.on_enabled()
        drivetrain.request_linear_velocity(2.0, BasicPriority.TELEOP, "teleop")

        drivetrain.on_disabled()
        left_v, right_v = test_io.get_last_commanded_voltages()
        assert left_v == 0.0 and right_v == 0.0, "Should zero on disable"
