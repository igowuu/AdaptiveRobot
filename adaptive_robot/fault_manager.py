import wpilib

from adaptive_robot.interfaces.faultable import Faultable
from adaptive_robot.interfaces.schedulable_interface.schedulable_subscheduler import SchedulableSubscheduler
from adaptive_robot.interfaces.telemetry_interface.telemetry_subscheduler import TelemetrySubscheduler
from adaptive_robot.interfaces.tunable_interface.tunable_subscheduler import TunableSubscheduler
from adaptive_robot.autonomous.action_scheduler import ActionScheduler
from adaptive_robot.faults.fault_scheduler import FaultLogger
from adaptive_robot.faults.faults import Fault, FaultException, FaultSeverity


class FaultManager(Faultable):
    """
    Runs all schedulers in addition to handling and logging raised Fault
    objects each iteration.
    """
    def __init__(
        self, 
        schedulable_subscheduler: SchedulableSubscheduler,
        telemetry_subscheduler: TelemetrySubscheduler,
        tunable_subscheduler: TunableSubscheduler,
        action_scheduler: ActionScheduler, 
        fault_logger: FaultLogger
    ) -> None:
        self._schedulable_subscheduler = schedulable_subscheduler
        self._telemetry_subscheduler = telemetry_subscheduler
        self._tunable_subscheduler = tunable_subscheduler

        self._action_scheduler = action_scheduler
        self._fault_logger = fault_logger
        self._faults_this_iter: list[Fault] = []
    
    def run(self, enabled: bool) -> bool:
        """
        Runs all schedulers and logs faults each iteration.  

        :returns True: If the robot should disable (CRITICAL fault raised).
        """
        try:
            self._tunable_subscheduler.run()
            self._telemetry_subscheduler.run()
            self._schedulable_subscheduler.run(enabled)
            self._action_scheduler.run()
            return False
        except FaultException as e:
            self._handle_fault(e)
            return e.fault.severity == FaultSeverity.CRITICAL
        finally:
            self._log_faults()
    
    def _handle_fault(self, fault_exception: FaultException) -> None:
        """
        Adds a Fault object to the internal list and reports the Fault to terminal via wpilib.
        """
        fault = fault_exception.fault
        self._faults_this_iter.append(fault)
        
        if fault.severity == FaultSeverity.CRITICAL:
            wpilib.reportError(f"[CRITICAL] Fault raised: {fault.description}")
        elif fault.severity == FaultSeverity.ERROR:
            wpilib.reportWarning(f"[ERROR] Fault raised: {fault.description}")
        else:
            wpilib.reportWarning(f"[WARNING] Fault raised: {fault.description}")
    
    def _log_faults(self) -> None:
        """
        Logs all faults this iteration to a file.
        """
        self._fault_logger.run(self._faults_this_iter)
        self._faults_this_iter.clear()
