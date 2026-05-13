import wpilib

from adaptive_robot.faults.faults import FaultSeverity, FaultException
from adaptive_robot.interfaces.subscheduler import Subscheduler
from adaptive_robot.interfaces.schedulable_interface.schedulable import Schedulable



class SchedulableSubscheduler(Subscheduler):
    """
    Subscheduler that handles health and lifecycle execution.
    """
    def __init__(self, schedulables: list[Schedulable], default_fault_threshold: int) -> None:
        self.schedulables = schedulables
        self._consecutive_faults: dict[Schedulable, int] = {
            schedulable: 0 for schedulable in self.schedulables
        }
        self.previously_enabled = False

        for schedulable in schedulables:
            schedulable.fault_threshold = default_fault_threshold

    def _record_schedulable_fault(self, schedulable: Schedulable) -> None:
        """
        Records an ERROR fault for a schedulable and marks it unhealthy if threshold is reached.

        :raises FaultException: If on_faulted_init raises an Exception.
        """
        self._consecutive_faults[schedulable] += 1
        
        if self._consecutive_faults[schedulable] >= schedulable.fault_threshold:
            wpilib.reportWarning(
                f"Schedulable {schedulable.__class__.__name__} reached consecutive fault threshold ({schedulable.fault_threshold}). "
                f"Marking Schedulable as unhealthy (faulted)."
            )
            schedulable.set_health(False)
            try:
                schedulable.on_faulted_init()
            except Exception as e:
                message = f"Error occured when calling on_faulted_init: {e}"
                self.raise_fault(schedulable, FaultSeverity.CRITICAL, message, e)

    def _reset_schedulable_faults(self, schedulable: Schedulable) -> None:
        """
        Resets fault counter for a schedulable after successful execution per iteration.
        """
        self._consecutive_faults[schedulable] = 0

    def _execute_schedulables(self, enabled: bool) -> None:
        """
        Executes healthy schedulables or calls on_faulted() for unhealthy schedulables if enabled.  
        Tracks consecutive faults and automatically marks schedulables unhealthy after
        threshold if faulted.

        :raises FaultException: Upon execute() or on_faulted_periodic() raising an Exception.
        """
        for schedulable in self.schedulables:
            if schedulable.locked:
                continue
            try:
                if enabled:
                    if schedulable.is_healthy():
                        schedulable.execute()
                        # Reset fault counter on successful execution
                        self._reset_schedulable_faults(schedulable)
                    else:
                        schedulable.on_faulted_periodic()
            except FaultException as e:
                if e.fault.severity != FaultSeverity.WARNING:
                    self._record_schedulable_fault(schedulable)
                raise
            except Exception as e:
                message = f"CRITICAL {schedulable.__class__.__name__} raised non-FaultException error: {e}"
                self._record_schedulable_fault(schedulable)
                self.raise_fault(schedulable, FaultSeverity.CRITICAL, message, e)

    def _call_activation_methods(self, enabled: bool) -> None:
        """
        Calls the activation methods (on_enabled and on_disabled) upon the robot
        having just enabled or disabled.

        :raises FaultException: Upon on_enabled() raising.
        """
        # If switch from disabled to enabled
        if not self.previously_enabled and enabled:
            for schedulable in self.schedulables:
                if schedulable.locked:
                    continue
                try:
                    schedulable.on_enabled()
                except FaultException as e:
                    if e.fault.severity != FaultSeverity.WARNING:
                        self._record_schedulable_fault(schedulable)
                    raise
                except Exception as e:
                    message = f"CRITICAL {schedulable.__class__.__name__} raised non-FaultException error: {e}"
                    self._record_schedulable_fault(schedulable)
                    self.raise_fault(schedulable, FaultSeverity.CRITICAL, message, e)
        
        # If switch from enabled to disabled
        if self.previously_enabled and not enabled:
            for schedulable in self.schedulables:
                if schedulable.locked:
                    continue
                try:
                    schedulable.on_disabled()
                
                except FaultException as e:
                    if e.fault.severity != FaultSeverity.WARNING:
                        self._record_schedulable_fault(schedulable)
                    raise
                except Exception as e:
                    message = f"CRITICAL {schedulable.__class__.__name__} raised non-FaultException error: {e}"
                    self._record_schedulable_fault(schedulable)
                    self.raise_fault(schedulable, FaultSeverity.CRITICAL, message, e)

    def run(self, enabled: bool) -> None:
        """
        Executes the subscheduler each iteration if enabled.
        Executes healthy schedulables, and handles faults regardless of being enabled or not.

        Subscheduler-specific Faults raised will not crash the robot immediately unless CRITICAL.
        Any other Exceptions raised by the schedulable will crash the robot (CRITICAL Faults).

        :raises FaultException: Upon any error when executing state and schedulers.
        """
        try:
            self._call_activation_methods(enabled)
            self._execute_schedulables(enabled)
        finally:
            self.previously_enabled = enabled
    
    def reset_all_schedulable_health(self) -> None:
        """
        Resets all schedulables to HEALTHY and clears their fault counters.
        Called when the robot is disabled to prepare for next match.
        """
        for schedulable in self.schedulables:
            schedulable.set_health(True)
            self._consecutive_faults[schedulable] = 0
