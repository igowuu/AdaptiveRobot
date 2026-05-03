from adaptive_robot import AdaptiveComponent, BasicPriority, RequestArbitrator

from wpilib import RobotController
from wpimath.units import percent

from components.intake.intake_io.io_base import IntakeIOBase


class Intake(AdaptiveComponent):
    """
    Represents a subsystem containing rollers that fling fuel into the bason.
    """
    def __init__(self, io: IntakeIOBase) -> None:
        self.io = io
        self.percent_controller = RequestArbitrator()

    def request_percent(
        self,
        percent: percent,
        priority: BasicPriority, 
        source: str = "unknown"
    ) -> None:
        """
        Requests a velocity to the intake's percent controller.
        """
        self.percent_controller.request(percent, priority.value, source)

    def _safe_defaults(self) -> None:
        """
        Directly commands safe default values to the intake.
        """
        self.io.set_voltage(0.0)

    def on_enabled(self) -> None:
        """
        Method called once when the robot enters a enabled state.
        """
        self._safe_defaults()

    def on_disabled(self) -> None:
        """
        Method called once when the robot enters a disabled state.
        """
        self._safe_defaults()
    
    def on_faulted_init(self) -> None:
        """
        Runs once when the component first becomes unhealthy.
        """
        self._safe_defaults()
    
    def on_faulted_periodic(self) -> None:
        """
        Runs each iteration when the component is unhealthy.
        """
        self._safe_defaults()

    def publish_telemetry(self) -> None:
        """
        Runs each iteration before component execution, regardless of component health.
        """
        self.publish_value("Intake/sensorData/voltage", self.io.get_voltage())
        resolved = self.percent_controller.resolve()
        self.publish_value("Intake/resolvedPercent/percent", resolved.value)
        self.publish_value("Intake/resolvedPercent/source", resolved.source)
        self.publish_value("Intake/resolvedPercent/priority", resolved.priority)

    def execute(self) -> None:
        """
        Directly moves the robot each iteration if the component is healthy.
        """
        resolved_percent = self.percent_controller.resolve().value
        self.io.set_voltage(resolved_percent * RobotController.getBatteryVoltage())

        self.io.update()
