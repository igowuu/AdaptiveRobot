from adaptive_robot import AdaptiveRobot

from wpilib import Joystick

from components.drivetrain.drivetrain import Drivetrain
from components.drivetrain.drivetrain_controller import DrivetrainController
from components.drivetrain.drivetrain_io.real_io import RealDrivetrainIO
from components.drivetrain.drivetrain_io.simulated_io import SimulatedDrivetrainIO

from components.arm.arm import Arm
from components.arm.arm_controller import ArmController
from components.arm.arm_io.real_io import RealArmIO
from components.arm.arm_io.simulated_io import SimulatedArmIO

from components.intake.intake import Intake
from components.intake.intake_controller import IntakeController
from components.intake.intake_io.real_io import RealIntakeIO
from components.intake.intake_io.simulated_io import SimulatedIntakeIO

from actions.trajectories.taxi import taxi_drive


class MyRobot(AdaptiveRobot):
    def onRobotInit(self) -> None:
        super().__init__()

        self.controller = Joystick(0)

        if self.isSimulation():
            self.drivetrain_io = SimulatedDrivetrainIO()
            self.arm_io = SimulatedArmIO()
            self.intake_io = SimulatedIntakeIO()
        else:
            self.drivetrain_io = RealDrivetrainIO()
            self.arm_io = RealArmIO()
            self.intake_io = RealIntakeIO()
        
        self.drivetrain = Drivetrain(self.drivetrain_io)
        self.arm = Arm(self.arm_io)
        self.intake = Intake(self.intake_io)

        self.drivetrain_controller = DrivetrainController(self.drivetrain, self.controller)
        self.arm_controller = ArmController(self.arm, self.controller)
        self.intake_controller = IntakeController(self.intake, self.controller)

    def onAutonomousInit(self) -> None:
        self.schedule_action(taxi_drive(self.drivetrain), "taxi_drive")
