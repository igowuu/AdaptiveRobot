import logging
from enum import Enum, auto

from wpilib import Joystick, DriverStation

from components.arm import Arm
from utils.fsm import FiniteStateMachine

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component import AdaptiveComponent

from config.constants import ArmConst

logger = logging.getLogger(__name__)

class ArmCommand(Enum):
    """Holds all possible high-level arm commands."""
    IDLE = auto()
    UP = auto()
    DOWN = auto()


class ArmControl(AdaptiveComponent):
    def __init__(self, robot: "AdaptiveRobot", arm: Arm, controller: Joystick) -> None:
        super().__init__(robot)

        self.arm = arm
        self.fsm = FiniteStateMachine()
        
        self.controller = controller
        
        self.command = ArmCommand.IDLE

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
            name="move_up",
            conditional=self._should_move_up,
            on_update=self._on_up_update,
            on_enable=self._on_up_enable,
            on_disable=self._on_up_disable
        )

        self.fsm.add_state(
            name="move_down",
            conditional=self._should_move_down,
            on_update=self._on_down_update,
            on_enable=self._on_down_enable,
            on_disable=self._on_down_disable
        )
    
    # General helpers
    def _apply_pct_output(self, pct_output: float) -> None:
        """
        Apply a percentage output to the arm motor(s) from [-1.0, 1.0].
        """
        self.arm.move_arm(pct_output)

    # Conditions
    def _should_move_up(self) -> bool:
        return self.command == ArmCommand.UP

    def _should_move_down(self) -> bool:
        return self.command == ArmCommand.DOWN

    # On_update methods
    def _on_idle_update(self) -> None:
        self.arm.stop()
        
    def _on_up_update(self) -> None:
        self._apply_pct_output(ArmConst.APPLIED_PCT_OUT)

    def _on_down_update(self) -> None:
        self._apply_pct_output(-ArmConst.APPLIED_PCT_OUT)

    # On_enable methods
    def _on_idle_enable(self) -> None:
        logger.info("[ARM] Idle")
        self.arm.stop()
    
    def _on_up_enable(self) -> None:
        logger.info("[ARM] Moving up")

    def _on_down_enable(self) -> None:
        logger.info("[ARM] Moving down")

    # On_disable methods
    def _on_idle_disable(self) -> None:
        logger.debug("[ARM] Leaving idle")

    def _on_up_disable(self) -> None:
        logger.debug("[ARM] Stopped moving up")

    def _on_down_disable(self) -> None:
        logger.debug("[ARM] Stopped moving down")

    # Main executable
    def execute(self) -> None:
        """
        Execute the FSM to update the arm's state.
        This is automatically called by AdaptiveRobot.
        """
        # No commands can be made outside of teleop mode.
        if not DriverStation.isTeleopEnabled():
            self.command = ArmCommand.IDLE
            self.fsm.execute()
            return
        
        if self.controller.getRawButton(1):
            self.command = ArmCommand.UP
        elif self.controller.getRawButton(2):
            self.command = ArmCommand.DOWN
        else:
            self.command = ArmCommand.IDLE
            
        self.fsm.execute()
