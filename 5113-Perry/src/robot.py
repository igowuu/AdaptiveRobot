# Perry - 5113 FRC code
# Copyright (c) 2026 Jacob Taylor (igowu) <https://github.com/igowuu>
# Code from: <https://github.com/igowuu/AdaptiveRobot.git>

# Licensed under the MIT License.
# See https://opensource.org/licenses/MIT for details.

import logging

from wpilib import Joystick
from wpimath.geometry import Pose2d

from components.drivetrain import Drivetrain, DrivePriority
from components.arm import Arm
from components.intake import Intake

from controls.arm_control import ArmControl
from controls.intake_control import IntakeControl

from utils.dashboard import Dashboard

from autonomous.routines.test_drive import TestDriveRoutine
from autonomous.manager import AutoManager

from adaptive_robot.adaptive_robot import AdaptiveRobot

logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s - %(message)s'
)

logger = logging.getLogger(__name__)

class Perry(AdaptiveRobot):
    """
    robot.py initializes and orchestrates all files.
    """

    def onRobotInit(self) -> None:
        self.lstick = Joystick(0)
        self.rstick = Joystick(1)

        self.drivetrain = Drivetrain(self)

        self.arm = Arm(self)
        self.arm_control = ArmControl(self, self.arm, self.lstick)
        
        self.intake = Intake(self)
        self.intake_control = IntakeControl(self, self.intake, self.lstick)

        self.dashboard = Dashboard(self, self.drivetrain)

        self.auto_manager = AutoManager(self)

    def disabledPeriodic(self) -> None:
        self.drivetrain.stop()
        self.arm.stop()
        self.intake.stop()

    def autonomousInit(self):
        # This is a simple test routine to demonstrate the drive_to_odometry auton step.
        routine = TestDriveRoutine(
            drivetrain=self.drivetrain,
            target_pose=Pose2d(1.0, -1.0, 0.0)
        )
        self.auto_manager.start(routine)

    def teleopPeriodic(self) -> None:
        forward_speed_pct = -self.lstick.getY()
        rotation_speed_pct = self.rstick.getX()

        self.drivetrain.request_forward_percent(forward_speed_pct, DrivePriority.TELEOP, "teleop")
        self.drivetrain.request_rotation_percent(rotation_speed_pct, DrivePriority.TELEOP, "teleop")
