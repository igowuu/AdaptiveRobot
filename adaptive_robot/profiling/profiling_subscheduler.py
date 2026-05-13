from adaptive_robot.interfaces.subscheduler import Subscheduler
from adaptive_robot.profiling.profiling_logger import ProfilingLogger
from adaptive_robot.profiling.profiling_models import ProfilingContext
from adaptive_robot.profiling.profile_method import set_profiling_context

from wpimath.units import seconds


class ProfilingSubscheduler(Subscheduler):
    """
    Subscheduler that tracks method execution statistics and logs periodically.
    Logs statistics to file when robot becomes disabled or on periodic intervals.
    """
    def __init__(
        self, 
        logging_folder: str, 
        logging_enabled: bool, 
        logging_frequency: seconds,
        period: seconds
    ) -> None:
        self.context = ProfilingContext()
        set_profiling_context(self.context)

        self.logger = ProfilingLogger(logging_folder)
        self.logging_enabled = logging_enabled
        
        self._previously_disabled = False
        self._iterations_since_log = 0
        self._logging_interval = self._get_logging_interval(logging_frequency, period)
    
    def _get_logging_interval(self, logging_frequency: seconds, period: seconds) -> int:
        """
        Returns the logging interval (how many iterations before logging), 
        given a frequency and period in seconds.
        """
        return int(logging_frequency / period)

    def _is_transitioned_disabled(self, enabled: bool) -> bool:
        """
        Returns True if the robot transitioned to a disabled state for the first time.
        """
        if self._previously_disabled and enabled:
            self._previously_disabled = False
        if not self._previously_disabled and not enabled:
            self._previously_disabled = True
            return True

        return False

    def run(self, enabled: bool) -> None:
        """
        Processes method profiles and logs periodically or on disable transition.
        """
        if not self.logging_enabled:
            return

        self._iterations_since_log += 1

        if self._iterations_since_log >= self._logging_interval:
            self.logger.log_profiles(self.context.method_profiles, append=False)
            self._iterations_since_log = 0

        if self._is_transitioned_disabled(enabled):
            self.logger.log_profiles(self.context.method_profiles, append=True)
