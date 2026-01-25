from wpimath.geometry import Pose2d

from components.drivetrain import Drivetrain

from autonomous.steps.drive_to_pose import DriveToPoseStep
from autonomous.routines.routine_base import SequentialRoutine

from config.constants import RobotConst


class TestDriveRoutine(SequentialRoutine):
    """
    Test drive to showcase the drive_to_odometry step.
    """

    def __init__(self, drivetrain: Drivetrain, target_pose: Pose2d) -> None:
        super().__init__([
            DriveToPoseStep(
                drivetrain=drivetrain,
                target_pose=target_pose,
                max_linear_mps=RobotConst.MAX_SPEED_MPS,
                max_accel_mps_squared=RobotConst.MAX_ACCEL_MPS_SQUARED
            )
        ])
