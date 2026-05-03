from adaptive_robot import AdaptiveComponent, BasicPriority, RequestArbitrator

from wpilib import RobotController
from wpimath.units import meters_per_second

from components.shooter.shooter_io.io_base import ShooterIOBase
from components.shooter.shooter_constants import ShooterConstants


class Shooter(AdaptiveComponent):
    """
    Represents a subsystem containing a full-length robot drum with a static hood.
    """
    def __init__(self, io: ShooterIOBase) -> None:
        self.io = io
        self.velocity_controller = RequestArbitrator()

    def request_velocity(
        self,
        percent: meters_per_second,
        priority: BasicPriority, 
        source: str = "unknown"
    ) -> None:
        """
        Requests a velocity to the shooters velocity controller.
        """
        self.velocity_controller.request(percent, priority.value, source)

    def _safe_defaults(self) -> None:
        """
        Directly commands safe default values to the Shooter.
        """
        self.io.set_voltage(0.0)

    def on_enabled(self) -> None:
        """
        Runs once when the robot enters an enabled mode.
        """
        self._safe_defaults()

    def on_disabled(self) -> None:
        """
        Runs once when the robot enters a disabled mode.
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
        self.publish_value("Shooter/sensorData/voltage", self.io.get_voltage())
        resolved = self.velocity_controller.resolve()
        self.publish_value("Shooter/resolvedVelocity/velocity", resolved.value)
        self.publish_value("Shooter/resolvedVelocity/source", resolved.source)
        self.publish_value("Shooter/resolvedVelocity/priority", resolved.priority)

    def execute(self) -> None:
        """
        Directly moves the robot each iteration if the component is healthy.
        """
        resolved_velocity = self.velocity_controller.resolve().value  # Linear velocity in m/s

        # Convert linear velocity in mps to angular velocity in radps
        angular_velocity = resolved_velocity / ShooterConstants.FLYWHEEL_RADIUS
        percent = angular_velocity / ShooterConstants.MAX_VELOCITY
        self.io.set_voltage(percent * RobotController.getBatteryVoltage())

        self.io.update()
