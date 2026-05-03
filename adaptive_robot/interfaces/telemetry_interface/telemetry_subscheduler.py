from adaptive_robot.telemetry.telemetry import TelemetryPublisher
from adaptive_robot.telemetry.struct_telemetry import TelemetryStructPublisher
from adaptive_robot.interfaces.subscheduler import Subscheduler
from adaptive_robot.interfaces.telemetry_interface.telemetry_publishable import TelemetryContext, TelemetryPublishable
from adaptive_robot.faults.faults import FaultException, FaultSeverity


class TelemetrySubscheduler(Subscheduler):
    """
    Subscheduler that updates all telemetry each iteration for subclasses of TelemetryPublishable.
    """
    def __init__(
        self,
        telemetry_publisher: TelemetryPublisher,
        struct_telemetry_publisher: TelemetryStructPublisher,
        telemetry_publishables: list[TelemetryPublishable]
    ) -> None:
        """
        Creates one shared TelemetryContext and injects it into all TelemetryPublisher objects.
        """
        self.telemetry_publishables = telemetry_publishables
        self._context = TelemetryContext(telemetry_publisher, struct_telemetry_publisher)

        for telemetry_publishable in telemetry_publishables:
            telemetry_publishable.telemetry_context = self._context

    def _publish_telemetry(self) -> None:
        """
        Publishes telemetry for all components.

        :raises FaultException: Upon publish_telemetry raising an Exception.
        """
        for telemetry_publishable in self.telemetry_publishables:
            try:
                telemetry_publishable.publish_telemetry()
            except FaultException:
                raise
            except Exception as e:
                message = f"Telemetry error for {telemetry_publishable.__class__.__name__}: {e}"
                self.raise_fault(None, FaultSeverity.WARNING, message, e)

    def run(self) -> None:
        """
        Executes the subscheduler each iteration if enabled.
        Updates all telemetry and calls the publishe_telemetry lifecycle method each iteartion.

        :raises FaultException: Upon any error when executing state and schedulers.
        """
        self._publish_telemetry()
