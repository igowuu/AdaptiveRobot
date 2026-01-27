from wpilib import Timer

from wpimath.controller import LTVUnicycleController
from wpimath.geometry import Pose2d
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig, Trajectory
from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.units import meters_per_second, meters_per_second_squared

from components.drivetrain import Drivetrain, DrivePriority

from autonomous.steps.step_base import Step

from config.constants import RobotConst, DrivetrainPID


class DriveToPoseStep(Step):
    """
    Drives the bot to a pose using trajectory and a LTV controller.
    """

    def __init__(
        self,
        drivetrain: Drivetrain,
        target_pose: Pose2d,
        max_linear_mps: meters_per_second,
        max_accel_mps_squared: meters_per_second_squared
    ) -> None:
        self.drivetrain = drivetrain
        self.target_pose = target_pose

        self.controller = LTVUnicycleController(
            DrivetrainPID.B,
            DrivetrainPID.ZETA
        )

        self.kinematics = DifferentialDriveKinematics(
            RobotConst.TRACK_WIDTH
        )

        self.config = TrajectoryConfig(max_linear_mps, max_accel_mps_squared)
        self.config.setKinematics(self.kinematics)

        self.timer = Timer()
        self.trajectory = Trajectory()

        self._round_digits = 3

    def start(self) -> None:
        start_pose = self.drivetrain.get_pose()

        self.trajectory = TrajectoryGenerator.generateTrajectory(
            start=start_pose,
            interiorWaypoints=[],
            end=self.target_pose,
            config=self.config
        )

        self.timer.reset()
        self.timer.start()

    def get_telemetry(self) -> dict[str, object]:
        t = self.timer.get()

        state = self.trajectory.sample(min(t, self.trajectory.totalTime()))
        current_pose = self.drivetrain.get_pose()
        chassis_speeds = self.controller.calculate(
            currentPose=current_pose,
            poseRef=state.pose,
            linearVelocityRef=state.velocity,
            angularVelocityRef=state.velocity * state.curvature
        )

        return {
            "time": t,
            "current_pose": current_pose,
            "trajectory_pose": state.pose,
            "trajectory_velocity": state.velocity,
            "trajectory_angular_velocity": state.velocity * state.curvature,
            "commanded_vx": chassis_speeds.vx,
            "commanded_omega": chassis_speeds.omega
        }

    def update(self) -> None:
        current_pose = self.drivetrain.get_pose()
        t = self.timer.get()
        state = self.trajectory.sample(t)

        chassis_speeds = self.controller.calculate(
            currentPose=current_pose,
            poseRef=state.pose,
            linearVelocityRef=state.velocity,
            angularVelocityRef=state.velocity * state.curvature
        )

        self.drivetrain.request_forward_percent(
            forward_speed=chassis_speeds.vx / RobotConst.MAX_SPEED_MPS, 
            priority=DrivePriority.AUTO, 
            source="auto"
        )
        self.drivetrain.request_rotation_percent(
            rotation_speed=chassis_speeds.omega / RobotConst.MAX_SPEED_RADPS, 
            priority=DrivePriority.AUTO, 
            source="auto"
        )

    def is_finished(self) -> bool:
        return self.timer.get() >= self.trajectory.totalTime()

    def stop(self) -> None:
        self.drivetrain.stop()
