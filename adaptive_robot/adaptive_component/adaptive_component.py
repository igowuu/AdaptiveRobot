from adaptive_robot.interfaces.faultable import Faultable
from adaptive_robot.interfaces.schedulable_interface.schedulable import Schedulable
from adaptive_robot.interfaces.telemetry_interface.telemetry_publishable import TelemetryPublishable
from adaptive_robot.interfaces.tunable_interface.tunable_publishable import TunablePublishable


class AdaptiveComponent(Faultable, Schedulable, TelemetryPublishable, TunablePublishable):
    """
    This base class defines the execution and telemetry lifecycle for all robot components.
    AdaptiveRobot implements the Faultable, Schedulable, TelemetryPublishable, and 
    TunablePublishable interfaces.
    """
    def __init_subclass__(cls) -> None:
        """
        Auto-initializes interface contexts when a subclass is created.
        This ensures users don't have to manually initialize interfaces.
        """
        super().__init_subclass__()
