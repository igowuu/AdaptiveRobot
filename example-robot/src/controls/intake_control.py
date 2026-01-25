import logging
from enum import Enum, auto

from wpilib import Joystick, DriverStation

from components.intake import Intake
from utils.fsm import FiniteStateMachine

from adaptive_robot.adaptive_robot import AdaptiveComponent, AdaptiveRobot

from config.constants import IntakeConst

logger = logging.getLogger(__name__)

class IntakeCommand(Enum):
    """Holds all possible high-level intake commands."""
    IDLE = auto()
    GRAB = auto()
    RELEASE = auto()


class IntakeControl(AdaptiveComponent):
    def __init__(self, robot: "AdaptiveRobot", intake: Intake, lstick: Joystick) -> None:
        super().__init__(robot)

        self.intake = intake
        self.fsm = FiniteStateMachine()

        self.lstick = lstick

        self.command = IntakeCommand.IDLE
        
        self._setup_states()

    def _setup_states(self) -> None:
        # NOTE: State evaluation order defines priority (top = highest)
        self.fsm.set_default_state(
            name="idle",
            on_update=self._on_idle_update,
            on_enable=self._on_idle_enable,
            on_disable=self._on_idle_disable,
        )

        self.fsm.add_state(
            name="grab",
            conditional=self._should_grab,
            on_update=self._on_grab_update,
            on_enable=self._on_grab_enable,
            on_disable=self._on_grab_disable
        )

        self.fsm.add_state(
            name="release",
            conditional=self._should_release,
            on_update=self._on_release_update,
            on_enable=self._on_release_enable,
            on_disable=self._on_release_disable
        )
    
    # General helpers
    def _apply_pct_output(self, pct_output: float) -> None:
        """
        Apply a percentage output to the intake motor from [-1.0, 1.0].
        """
        self.intake.move_intake(pct_output)

    # Conditions
    def _should_grab(self) -> bool:
        return self.command == IntakeCommand.GRAB

    def _should_release(self) -> bool:
        return self.command == IntakeCommand.RELEASE

    # On_update methods
    def _on_idle_update(self) -> None:
        self.intake.stop()
        
    def _on_grab_update(self) -> None:
        self._apply_pct_output(IntakeConst.MAX_PCT_OUTPUT)

    def _on_release_update(self) -> None:
        self._apply_pct_output(-IntakeConst.MAX_PCT_OUTPUT)

    # On_enable methods
    def _on_idle_enable(self) -> None:
        logger.info("[INTAKE] Idle")
        self.intake.stop()

    def _on_grab_enable(self) -> None:
        logger.info("[INTAKE] Grabbing")

    def _on_release_enable(self) -> None:
        logger.info("[INTAKE] Releasing")

    # On_disable methods
    def _on_idle_disable(self) -> None:
        logger.debug("[INTAKE] Leaving idle")

    def _on_grab_disable(self) -> None:
        logger.debug("[INTAKE] Stopped grabbing")

    def _on_release_disable(self) -> None:
        logger.debug("[INTAKE] Stopped releasing")

    # Main executable
    def execute(self) -> None:
        """
        Execute the FSM to update the intake's state.
        This is automatically called by AdaptiveRobot.
        """
        # No commands can be made outside of teleop mode.
        if not DriverStation.isTeleopEnabled():
            self.command = IntakeCommand.IDLE
            self.fsm.execute()
            return
        
        if self.lstick.getRawButton(3):
            self.command = IntakeCommand.GRAB
        elif self.lstick.getRawButton(4):
            self.command = IntakeCommand.RELEASE
        else:
            self.command = IntakeCommand.IDLE

        self.fsm.execute()
