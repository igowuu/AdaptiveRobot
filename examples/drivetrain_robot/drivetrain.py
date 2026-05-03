from adaptive_robot import AdaptiveComponent, RequestArbitrator, BasicPriority

from drivetrain_io import DrivetrainIO

from wpimath.units import percent


class Drivetrain(AdaptiveComponent):
    def __init__(self) -> None:
        self.io = DrivetrainIO()
        self.linear_percent_controller = RequestArbitrator()
        self.angular_percent_controller = RequestArbitrator()
    
    def request_linear_percent(
        self, 
        linear_percent: percent, 
        priority: BasicPriority,
        source: str = "unknown"
    ) -> None:
        """
        Requests a percent to the drivetrains linear percent controller.

        :param linear_percent: The percent that will be added to the RequestArbitrator.
        :param priority: The priority given to the request, used when arbitrating.
        :param source: The name given to the request for telemetry.
        """
        self.linear_percent_controller.request(linear_percent, priority.value, source)

    def request_angular_percent(
        self, 
        angular_percent: percent, 
        priority: BasicPriority,
        source: str = "unknown"
    ) -> None:
        """
        Requests a percent to the drivetrains linear percent controller.

        :param angular_percent: The percent that will be added to the RequestArbitrator.
        :param priority: The priority given to the request, used when arbitrating.
        :param source: The name given to the request for telemetry.
        """
        self.angular_percent_controller.request(angular_percent, priority.value, source)
    
    def _safe_defaults(self) -> None:
        """
        Directly commands safe defaults to the IO.
        """
        self.io.drive(0.0, 0.0)

    def on_enabled(self) -> None:
        """
        Hook that will be called once when the robot transitions to an enabled state.
        Safe defaults are commanded here to ensure that motors start with zero movement.
        """
        self._safe_defaults()
    
    def on_disabled(self) -> None:
        """
        Hook that will be called once when the robot transitions to a disabled state.
        Safe defaults are commanded here to ensure that motors end with zero movement.
        """
        self._safe_defaults()
    
    def on_faulted_init(self) -> None:
        """
        Hook that will be called once when the component enters an unhealthy state.
        Safe defaults are commanded here to ensure that motors command zero voltage upon
        an incorrectly working component.
        """
        self._safe_defaults()
    
    def on_faulted_periodic(self) -> None:
        """
        Hook that will be called each iteration when the component enters an unhealthy state.
        Safe defaults are commanded here to ensure that motors command zero voltage upon
        an incorrectly working component.
        """
        self._safe_defaults()
    
    def publish_telemetry(self) -> None:
        """
        Hook that is called each iteration, regardless of component health.
        All telemetry that updates each iteration should ideally go here.
        """
        self.publish_value("Drive/isHealthy", self.is_healthy())
        self.publish_value("Drive/leftVoltage", self.io.get_left_voltage())
        self.publish_value("Drive/rightVoltage", self.io.get_right_voltage())

    def execute(self) -> None:
        """
        Hook that is called each iteration upon a healthy component.
        """
        resolved_linear = self.linear_percent_controller.resolve()
        resolved_angular = self.angular_percent_controller.resolve()

        self.io.drive(resolved_linear.value, resolved_angular.value)
