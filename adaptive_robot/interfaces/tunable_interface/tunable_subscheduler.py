from adaptive_robot.faults.faults import FaultSeverity, FaultException
from adaptive_robot.interfaces.subscheduler import Subscheduler
from adaptive_robot.interfaces.tunable_interface.tunable_publishable import TunablePublishable, TunableContext


class TunableSubscheduler(Subscheduler):
    """
    Subscheduler that updates all tunable values and PID controllers each iteration.
    Manages a single shared TunableContext that all TunablePublishable objects inject into.
    """
    def __init__(self, tunable_publishables: list[TunablePublishable]) -> None:
        """
        Creates one shared TunableContext and injects it into all TunablePublishable objects.
        
        :param tunable_publishables: List of all objects implementing TunablePublishable.
        """
        self._context = TunableContext()

        for tunable_publishable in tunable_publishables:
            tunable_publishable.tunable_context = self._context

    def _update_tunables(self) -> None:
        """
        Updates all tunable values and PID controllers from the shared context.

        :raises FaultException: Upon an Exception being raised by tunables.
        """
        try:
            for tunable in self._context.tunables:
                tunable.update()
            for tunablePID in self._context.tunable_pids:
                tunablePID.update_from_tunables()
    
        except FaultException:
            raise

        except Exception as e:
            message = f"Tunable update error: {e}"
            self.raise_fault(None, FaultSeverity.WARNING, message, e)

    def run(self) -> None:
        """
        Executes the subscheduler each iteration.
        Updates all tunable values and PID controllers.
        """
        self._update_tunables()
