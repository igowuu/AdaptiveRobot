from enum import Enum, auto

from wpilib import Field2d, SmartDashboard

from wpimath.geometry import Pose2d
from wpimath.units import meters_per_second, radians_per_second
from wpimath.kinematics import DifferentialDriveOdometry, ChassisSpeeds, DifferentialDriveKinematics
from wpimath.controller import SimpleMotorFeedforwardMeters

from components.drivetrain.drivetrain_io.io_base import DrivetrainIOBase
from components.drivetrain.drivetrain_constants import (
    DriveConstants, 
    DriveLeftFF,
    DriveRightFF,
    DriveLeftPID,
    DriveRightPID
)

from adaptive_robot import AdaptiveComponent, AxisController, BasicPriority


class DriveMode(Enum):
    OPEN_LOOP = auto()
    CLOSED_LOOP = auto()


class Drivetrain(AdaptiveComponent):
    def __init__(self, io: DrivetrainIOBase) -> None:
        super().__init__()
        self.io = io

        self.odometry = DifferentialDriveOdometry(
            gyroAngle=self.io.get_angle(),
            leftDistance=self.io.get_left_distance(),
            rightDistance=self.io.get_right_distance()
        )

        self.kinematics = DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH)

        self.left_pid = self.tunablePID(
            kp=DriveLeftPID.KP,
            ki=DriveLeftPID.KI,
            kd=DriveLeftPID.KD,
            directory="Tunables/Drivetrain/leftPID"
        )
        self.right_pid = self.tunablePID(
            kp=DriveRightPID.KP,
            ki=DriveRightPID.KI,
            kd=DriveRightPID.KD,
            directory="Tunables/Drivetrain/rightPID"
        )

        self.left_ff = SimpleMotorFeedforwardMeters(
            kS=DriveLeftFF.KS,
            kV=DriveLeftFF.KV,
            kA=DriveLeftFF.KA
        )
        self.right_ff = SimpleMotorFeedforwardMeters(
            kS=DriveRightFF.KS,
            kV=DriveRightFF.KV,
            kA=DriveRightFF.KA
        )

        self.linear_velocity_controller = AxisController()
        self.angular_velocity_controller = AxisController()

        self.drive_mode = DriveMode.CLOSED_LOOP

        self.simulated_field = Field2d()
        SmartDashboard.putData("Field", self.simulated_field)

    def get_pose(self) -> Pose2d:
        """
        Returns the current estimated robot pose.
        """
        return self.odometry.getPose()

    def request_linear_velocity(
        self,
        linear_velocity: meters_per_second,
        priority: BasicPriority,
        source: str = "unknown"
    ) -> None:
        """
        Requests a velocity to the drivetrain's linear velocity controller.
        """
        self.linear_velocity_controller.request(linear_velocity, priority.value, source)

    def request_angular_velocity(
        self,
        angular_velocity: radians_per_second,
        priority: BasicPriority,
        source: str = "unknown"
    ) -> None:
        """
        Requests a velocity to the drivetrain's angular velocity controller.
        """
        self.angular_velocity_controller.request(angular_velocity, priority.value, source)

    def _safe_defaults(self) -> None:
        """
        Directly commands safe default values to the IO.
        """
        self.io.set_left_voltage(0.0)
        self.io.set_right_voltage(0.0)

    def on_enabled(self) -> None:
        """
        Runs once when the robot enters an enabled mode.
        """
        self._safe_defaults()

    def on_disabled(self) -> None:
        """
        Runs once when the robot enters a disabled mode.
        """
        self._safe_defaults()

    def on_faulted_init(self) -> None:
        """
        Runs once when the component becomes unhealthy.
        """
        self._safe_defaults()

    def on_faulted_periodic(self) -> None:
        """
        Runs each iteration when the component becomes unhealthy.
        """
        self._safe_defaults()

    def publish_telemetry(self) -> None:
        """
        Runs each iteration before execution regardless of component health.
        """
        self.publish_value("Drivetrain/isHealthy", self.is_healthy())
        self.publish_value("Drivetrain/driveMode", self.drive_mode.name)

        resolved_linear = self.linear_velocity_controller.resolve()
        self.publish_value("Drivetrain/resolvedLinear/value", resolved_linear.value)
        self.publish_value("Drivetrain/resolvedLinear/source", resolved_linear.source)
        self.publish_value("Drivetrain/resolv1edLinear/priority", resolved_linear.priority)

        resolved_angular = self.angular_velocity_controller.resolve()
        self.publish_value("Drivetrain/resolvedAngular/value", resolved_angular.value)
        self.publish_value("Drivetrain/resolvedAngular/source", resolved_angular.source)
        self.publish_value("Drivetrain/resolvedAngular/priority", resolved_angular.priority)

        self.publish_value("Drivetrain/sensorData/leftVelocity", self.io.get_left_velocity())
        self.publish_value("Drivetrain/sensorData/rightVelocity", self.io.get_right_velocity())
        self.publish_value("Drivetrain/sensorData/leftVoltage", self.io.get_left_voltage())
        self.publish_value("Drivetrain/sensorData/rightVoltage", self.io.get_right_voltage())

        self.publish_struct_value("Drivetrain/pose", self.get_pose())

    def _drive_closed_loop(
        self, 
        left_velocity: meters_per_second, 
        right_velocity: meters_per_second
    ) -> None:
        """
        Drives the component with PID + FF.
        """
        current_left_velocity = self.io.get_left_velocity()
        current_right_velocity = self.io.get_right_velocity()

        left_ff_volts = self.left_ff.calculate(current_left_velocity, left_velocity)
        right_ff_volts = self.right_ff.calculate(current_right_velocity, right_velocity)

        left_pid_volts = self.left_pid.calculate(current_left_velocity, left_velocity)
        right_pid_volts = self.right_pid.calculate(current_right_velocity, right_velocity)

        self.io.set_left_voltage(left_ff_volts + left_pid_volts)
        self.io.set_right_voltage(right_ff_volts + right_pid_volts)

    def _drive_open_loop(
        self,
        left_velocity: meters_per_second, 
        right_velocity: meters_per_second
    ) -> None:
        """
        Drives the component without PID + FF.
        """
        self.io.set_left_voltage(left_velocity / DriveConstants.MAX_LINEAR_SPEED)
        self.io.set_right_voltage(right_velocity / DriveConstants.MAX_LINEAR_SPEED)

    def execute(self) -> None:
        """
        Runs each iteration if the component is healthy.
        """
        linear_velocity = self.linear_velocity_controller.resolve().value
        angular_velocity = self.angular_velocity_controller.resolve().value
        chassis = ChassisSpeeds(linear_velocity, 0.0, angular_velocity)
        
        wheel_speeds = self.kinematics.toWheelSpeeds(chassis)
        wheel_speeds.desaturate(DriveConstants.MAX_LINEAR_SPEED)

        if self.drive_mode == DriveMode.OPEN_LOOP:
            self._drive_open_loop(wheel_speeds.left, wheel_speeds.right)
        else:
            self._drive_closed_loop(wheel_speeds.left, wheel_speeds.right)
        
        self.odometry.update(
            gyroAngle=self.io.get_angle(),
            leftDistance=self.io.get_left_distance(),
            rightDistance=self.io.get_right_distance()
        )

        self.simulated_field.setRobotPose(self.get_pose())

        self.io.update()
