from wpilib import Joystick, RobotState

from examples.comp_tank_drive.components.game_sim.game_piece_sim import GamePieceSim
from constants import JoystickButton

from adaptive_robot import Schedulable


class GamePieceController(Schedulable):
    """
    Controls the game piece simulator via teleoperation.
    """
    def __init__(self, game_piece_sim: GamePieceSim, controller: Joystick) -> None:
        self.game_piece_sim = game_piece_sim
        self.controller = controller

    def execute(self) -> None:
        """
        Method called each iteration if the component is healthy.
        """
        if not RobotState.isTeleop():
            return

        if self.controller.getRawButtonPressed(JoystickButton.GAME_SIM_CLEAR):
            self.game_piece_sim.clear_all_fuel()
        if self.controller.getRawButtonPressed(JoystickButton.GAME_SIM_SPAWN):
            self.game_piece_sim.spawn_fuel_line()
        if self.controller.getRawButton(JoystickButton.GAME_SIM_SHOOT):
            self.game_piece_sim.shoot_fuel()
