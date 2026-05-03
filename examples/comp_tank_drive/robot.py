from adaptive_robot import AdaptiveRobot

from wpilib import Joystick

from components.drivetrain.drivetrain import Drivetrain
from components.drivetrain.drivetrain_controller import DrivetrainController
from components.drivetrain.drivetrain_io.real_io import RealDrivetrainIO
from components.drivetrain.drivetrain_io.simulated_io import SimulatedDrivetrainIO

from components.intake_arm.intake_arm import IntakeArm
from components.intake_arm.intake_arm_controller import IntakeArmController
from components.intake_arm.intake_arm_io.real_io import RealArmIO
from components.intake_arm.intake_arm_io.simulated_io import SimulatedArmIO

from components.intake.intake import Intake
from components.intake.intake_controller import IntakeController
from components.intake.intake_io.real_io import RealIntakeIO
from components.intake.intake_io.simulated_io import SimulatedIntakeIO

from components.shooter.shooter import Shooter
from components.shooter.shooter_controller import ShooterController
from components.shooter.shooter_io.real_io import RealShooterIO
from components.shooter.shooter_io.simulated_io import SimulatedShooterIO

from components.game_sim.game_peice_sim import GamePieceSim
from components.game_sim.game_piece_controller import GamePieceController

from actions.taxi_drive import taxi_drive


class PerryV3(AdaptiveRobot):
    def __init__(self) -> None:
        super().__init__()

        self.controller = Joystick(0)

        if self.isSimulation():
            self.drivetrain_io = SimulatedDrivetrainIO()
            self.intake_arm_io = SimulatedArmIO()
            self.intake_io = SimulatedIntakeIO()
            self.shooter_io = SimulatedShooterIO()
        else:
            self.drivetrain_io = RealDrivetrainIO()
            self.intake_arm_io = RealArmIO()
            self.intake_io = RealIntakeIO()
            self.shooter_io = RealShooterIO()

        self.drivetrain = Drivetrain(self.drivetrain_io)
        self.intake_arm = IntakeArm(self.intake_arm_io)
        self.intake = Intake(self.intake_io)
        self.shooter = Shooter(self.shooter_io)

        self.drivetrain_controller = DrivetrainController(self.drivetrain, self.controller)
        self.intake_arm_controller = IntakeArmController(self.intake_arm, self.controller)
        self.intake_controller = IntakeController(self.intake, self.controller)
        self.shooter_controller = ShooterController(self.shooter, self.drivetrain, self.controller)

        if self.isSimulation():
            self.game_piece_sim = GamePieceSim(self.drivetrain, self.intake_arm_io, self.shooter_io)
            self.game_piece_controller = GamePieceController(self.game_piece_sim, self.controller)

    def onAutonomousInit(self) -> None:
        """
        Runs once when autonomous mode is entered.
        """
        self.schedule_action(taxi_drive(self.drivetrain), "taxi drive")
