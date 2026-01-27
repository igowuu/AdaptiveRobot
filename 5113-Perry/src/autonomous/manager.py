from typing import Optional

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent

from autonomous.routines.routine_base import SequentialRoutine


class AutoManager(AdaptiveComponent):
    """
    Owns and executes a single autonomous routine.
    Acts as the boundary between robot modes and autonomous logic.
    """
    def __init__(self, robot: AdaptiveRobot) -> None:
        super().__init__(robot)

        self._current_routine: Optional[SequentialRoutine] = None
        self._running: bool = False

    def start(self, routine: SequentialRoutine) -> None:
        """
        Start a new autonomous routine.
        If a routine is already running, it will be replaced.
        """
        if self._current_routine:
            self._current_routine.stop()

        self._current_routine = routine
        self._current_routine.start()
        self._running = True

    def publish_telemetry(self) -> None:
        self.publish_value("Auto/Running", self._running)
        self.publish_value("Auto/CurrentRoutine", str(type(self._current_routine)))

    def execute(self) -> None:
        """
        Update the active autonomous routine.
        Automatically called every loop by AdaptiveComponent.
        """
        if not self._running or self._current_routine is None:
            return

        self._current_routine.update()

        if self._current_routine.is_finished():
            self._current_routine.stop()
            self._running = False
            self._current_routine = None