from adaptive_robot import AdaptiveComponent, BasicPriority, RequestArbitrator

from wpilib import RobotController
from wpimath.units import percent

from simple_io import SimpleIO


class SimpleComponent(AdaptiveComponent):
    """
    Represents a very simple subsystem containing a single motor.
    """
    def __init__(self, io: SimpleIO) -> None:
        self.io = io
        self.percent_controller = RequestArbitrator()

    def request_percent(
        self,
        percent: percent,
        priority: BasicPriority, 
        source: str = "unknown"
    ) -> None:
        """
        Requests a velocity to the simple component's percent controller.

        :param percent: The percent that will be added to the RequestArbitrator.
        :param priority: The priority given to the request, used when arbitrating.
        :param source: The name given to the request for telemetry.
        """
        self.percent_controller.request(percent, priority.value, source)

    def execute(self) -> None:
        """
        Directly moves the robot each iteration if the component is healthy.
        """
        resolved_percent = self.percent_controller.resolve().value
        self.io.set_voltage(resolved_percent * RobotController.getBatteryVoltage())
