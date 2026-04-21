import logging
import pytest

from components.arm.arm import Arm
from components.arm.arm_io.testing_io import ArmTestIO

from adaptive_robot import BasicPriority
from adaptive_robot.adaptive_component.adaptive_component import ComponentContext
from adaptive_robot.telemetry.telemetry import TelemetryPublisher
from adaptive_robot.telemetry.struct_telemetry import TelemetryStructPublisher


@pytest.fixture
def test_io() -> ArmTestIO:
    """
    Creates a new test IO for each test.
    """
    return ArmTestIO()


@pytest.fixture
def arm(test_io: ArmTestIO) -> Arm:
    """
    Creates an arm component with test IO and context.
    """
    arm = Arm(io=test_io)

    telemetry = TelemetryPublisher()
    struct_telemetry = TelemetryStructPublisher()
    context = ComponentContext(
        telemetry=telemetry,
        struct_telemetry=struct_telemetry,
        logger=logging.getLogger("test_arm")
    )
    arm.context = context
    
    return arm


class TestArmRequestArbitration:
    """
    Tests for request arbitration with AxisController.
    """
    def test_single_request_resolution(
        self,
        arm: Arm
    ) -> None:
        """
        Tests that a single angle request correctly gets resolved.
        """
        arm.request_angle(1.0, BasicPriority.TELEOP, "teleop")

        angle_resolved = arm.angle_controller.resolve()
        
        assert angle_resolved.value == 1.0
        assert angle_resolved.source == "teleop"
        assert angle_resolved.priority == BasicPriority.TELEOP.value
    
    def test_higher_priority_override(
        self,
        arm: Arm
    ) -> None:
        """
        Test priority ordering with BasicPriority (safety > auto > teleop).
        """
        arm.request_angle(1.0, BasicPriority.TELEOP, "teleop")
        arm.request_angle(0.5, BasicPriority.AUTO, "auto")
        
        resolved = arm.angle_controller.resolve()
        assert resolved.value == 0.5
        assert resolved.source == "auto"

        arm.request_angle(0.0, BasicPriority.SAFETY, "safety")
        
        resolved = arm.angle_controller.resolve()
        assert resolved.value == 0.0
        assert resolved.source == "safety"
    
    def test_most_recent_wins(
        self,
        arm: Arm
    ) -> None:
        """
        When same priority, most recent request should win.
        """
        arm.request_angle(1.0, BasicPriority.TELEOP, "teleop_1")
        arm.request_angle(0.5, BasicPriority.TELEOP, "teleop_2")
        
        resolved = arm.angle_controller.resolve()
        assert resolved.value == 0.5


class TestArmSafeDefaults:
    """
    Tests that safe defaults are applied when necessary.
    """
    def test_safe_defaults_called_on_enabled(
        self,
        arm: Arm,
        test_io: ArmTestIO
    ) -> None:
        """
        Tests if on_enabled forces zero voltage.
        """
        test_io.set_voltage(6.0)

        arm.on_enabled()

        voltage = test_io.get_last_commanded_voltage()
        assert voltage == 0.0
    
    def test_safe_defaults_called_on_disabled(
        self,
        arm: Arm,
        test_io: ArmTestIO
    ) -> None:
        """
        Tests if on_disabled forces zero voltage.
        """
        test_io.set_voltage(6.0)
        
        arm.on_disabled()
        
        voltage = test_io.get_last_commanded_voltage()
        assert voltage == 0.0
    
    def test_safe_defaults_called_on_faulted_init(
        self,
        arm: Arm,
        test_io: ArmTestIO
    ) -> None:
        """
        Tests if on_faulted_init forces zero voltage.
        """
        test_io.set_voltage(6.0)
        
        arm.on_faulted_init()
        
        voltage = test_io.get_last_commanded_voltage()
        assert voltage == 0.0
    
    def test_safe_defaults_called_on_faulted_periodic(
        self,
        arm: Arm,
        test_io: ArmTestIO
    ) -> None:
        """
        Tests if on_faulted_periodic forces zero voltage.
        """
        test_io.set_voltage(6.0)
        
        arm.on_faulted_periodic()
        
        voltage = test_io.get_last_commanded_voltage()
        assert voltage == 0.0


class TestArmIntegration:
    """
    Simulates real scenarios for the arm.
    """
    def test_full_lifecycle(
        self,
        arm: Arm,
        test_io: ArmTestIO
    ) -> None:
        """
        Simulate robot disabled, request made during on_enabled, and then re-disabled.
        """
        arm.on_disabled()
        voltage = test_io.get_last_commanded_voltage()
        assert voltage == 0.0

        arm.on_enabled()
        arm.request_angle(1.0, BasicPriority.TELEOP, "teleop")

        arm.on_disabled()
        voltage = test_io.get_last_commanded_voltage()
        assert voltage == 0.0
