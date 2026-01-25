from enum import Enum

from rev import SparkMax, SparkMaxConfig
from navx import AHRS

from wpimath import applyDeadband
from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import DifferentialDriveOdometry, DifferentialDriveKinematics, ChassisSpeeds
from wpimath.units import percent, meters, meters_per_second

from adaptive_robot.adaptive_robot import AdaptiveComponent, AdaptiveRobot
from adaptive_robot.axis_request import AxisRequest, resolve_axis

from utils.math_utils import (
    clamp, 
    rotations_to_meters, 
    rps_to_mps, 
    per_minute_to_per_second, 
    degrees_to_radians
)
from config.constants import JoystickConst, RobotConst, DrivetrainConst, DrivetrainPID, DrivetrainFF


class DrivePriority(Enum):
    SAFETY = 3
    AUTO = 2
    TELEOP = 1


class Drivetrain(AdaptiveComponent):
    """
    This class declares the drivetrain to handle all linear motion and angular rotation of our bot.
    """

    def __init__(self, robot: "AdaptiveRobot") -> None:
        super().__init__(robot)

        BRUSHLESS = SparkMax.MotorType.kBrushless

        self.front_left_motor = SparkMax(11, BRUSHLESS)
        self.front_right_motor = SparkMax(13, BRUSHLESS)
        self.back_left_motor = SparkMax(12, BRUSHLESS)
        self.back_right_motor = SparkMax(14, BRUSHLESS)

        self.front_left_motor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kBrake),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.front_right_motor.configure(
            SparkMaxConfig().setIdleMode(SparkMaxConfig.IdleMode.kBrake).inverted(True),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.back_left_motor.configure(
            SparkMaxConfig()
                .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
                .follow(self.front_left_motor.getDeviceId(), False),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )
        self.back_right_motor.configure(
            SparkMaxConfig()
                .setIdleMode(SparkMaxConfig.IdleMode.kBrake)
                .follow(self.front_right_motor.getDeviceId(), True),
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters,
        )

        self.front_right_encoder = self.front_right_motor.getEncoder()
        self.front_left_encoder = self.front_left_motor.getEncoder()

        self.front_right_encoder.setPosition(0)
        self.front_left_encoder.setPosition(0)

        self.gyro = AHRS.create_spi()
        self.gyro.reset()

        self.odometry = DifferentialDriveOdometry(
            self.get_heading(),
            self.get_left_distance(),
            self.get_right_distance()
        )

        self.left_pid = PIDController(
            Kp=DrivetrainPID.LEFT_KP,
            Ki=DrivetrainPID.LEFT_KI,
            Kd=DrivetrainPID.LEFT_KD
        )
        self.left_ff = SimpleMotorFeedforwardMeters(
            kS=DrivetrainFF.LEFT_KS,
            kV=DrivetrainFF.LEFT_KV,
            kA=DrivetrainFF.LEFT_KA
        )

        self.right_pid = PIDController(
            Kp=DrivetrainPID.RIGHT_KP,
            Ki=DrivetrainPID.RIGHT_KI,
            Kd=DrivetrainPID.RIGHT_KD
        )
        self.right_ff = SimpleMotorFeedforwardMeters(
            kS=DrivetrainFF.RIGHT_KS,
            kV=DrivetrainFF.RIGHT_KV,
            kA=DrivetrainFF.RIGHT_KA
        )

        self.kinematics = DifferentialDriveKinematics(RobotConst.TRACK_WIDTH)

        self._forward_requests: list[AxisRequest] = []
        self._rotation_requests: list[AxisRequest] = []

    def get_left_distance(self) -> meters:
        """Returns the left encoder distance in meters traveled."""
        left_position = self.front_left_encoder.getPosition()
        return rotations_to_meters(
            motor_rotations=left_position,
            wheel_diameter=RobotConst.WHEEL_DIAMETER,
            motor_to_wheel_ratio=DrivetrainConst.GEAR_RATIO
        )

    def get_right_distance(self) -> meters:
        """Returns the right encoder distance in meters traveled."""
        right_position = self.front_right_encoder.getPosition()
        return rotations_to_meters(
            motor_rotations=right_position,
            wheel_diameter=RobotConst.WHEEL_DIAMETER,
            motor_to_wheel_ratio=DrivetrainConst.GEAR_RATIO
        )

    def get_heading(self) -> Rotation2d:
        """Returns the robot heading as a Rotation2d."""
        return Rotation2d(degrees_to_radians(self.gyro.getAngle()))

    def get_pose(self) -> Pose2d:
        """Returns the robot pose as a Pose2d."""
        return self.odometry.getPose()

    def reset_odometry(self, pose: Pose2d = Pose2d()) -> None:
        """Resets odometry to the specified pose."""
        self.odometry.resetPosition(
            self.get_heading(),
            self.get_left_distance(),
            self.get_right_distance(),
            pose
        )

    def reset_heading(self) -> None:
        """Resets the robot orientation to zero."""
        self.gyro.reset()

    def reset_drivetrain(self) -> None:
        """Resets encoders, the gyro heading, odometry, and PIDControllers."""
        self.front_left_encoder.setPosition(0)
        self.front_right_encoder.setPosition(0)

        self.reset_heading()
        self.reset_odometry()

        self.left_pid.reset()
        self.right_pid.reset()

    def stop(self) -> None:
        """Stops all drivetrain movement."""
        self._forward_requests.append(
            AxisRequest(0.0, DrivePriority.SAFETY.value, "safety")
        )
        self._rotation_requests.append(
            AxisRequest(0.0, DrivePriority.SAFETY.value, "safety")
        )

    def tank_drive_velocity(
        self, 
        desired_left_mps: meters_per_second, 
        desired_right_mps: meters_per_second
    ) -> None:
        """
        Drives each side of the drivetrain using closed-loop velocity control.

        Uses feedforward (kS, kV, kA) and PID to compute motor voltage.
        Inputs are wheel linear velocities in meters per second.
        """
        left_velocity = self.front_left_encoder.getVelocity()
        right_velocity = self.front_right_encoder.getVelocity()

        measured_left_mps = rps_to_mps(
            motor_rps=per_minute_to_per_second(left_velocity),
            wheel_diameter=RobotConst.WHEEL_DIAMETER,
            motor_to_wheel_ratio=DrivetrainConst.GEAR_RATIO
        )
        measured_right_mps = rps_to_mps(
            motor_rps=per_minute_to_per_second(right_velocity),
            wheel_diameter=RobotConst.WHEEL_DIAMETER,
            motor_to_wheel_ratio=DrivetrainConst.GEAR_RATIO
        )

        left_ff = self.left_ff.calculate(desired_left_mps)
        right_ff = self.right_ff.calculate(desired_right_mps)

        left_pid = self.left_pid.calculate(measured_left_mps, desired_left_mps)
        right_pid = self.right_pid.calculate(measured_right_mps, desired_right_mps)

        self.front_left_motor.setVoltage(
            clamp(left_ff + left_pid, -RobotConst.MAX_VOLTAGE, RobotConst.MAX_VOLTAGE)
        )
        self.front_right_motor.setVoltage(
            clamp(right_ff + right_pid, -RobotConst.MAX_VOLTAGE, RobotConst.MAX_VOLTAGE)
        )

    def request_forward_percent(
        self,
        forward_speed: percent,
        priority: DrivePriority,
        source: str
    ) -> None:
        """
        Adds a request for a forward speed percent. The request will execute or be rejected based on 
        the priority of the designated value in the Enum to avoid conflicting actions.
        
        :param forward_speed: Desired vertical pct output
        :param priority: Mode priority
        :param source: String to name the mode (for visuals only)
        """
        vertical = applyDeadband(forward_speed, JoystickConst.DEADBAND)
        self._forward_requests.append(
            AxisRequest(
                value=RobotConst.MAX_SPEED_MPS * vertical,
                priority=priority.value,
                source=source
            )
        )

    def request_rotation_percent(
        self,
        rotation_speed: percent,
        priority: DrivePriority,
        source: str
    ) -> None:
        """
        Adds a request for a rotation speed percent. The request will execute or be rejected based on 
        the priority of the designated value in the Enum to avoid conflicting actions.
        
        :param rotation_speed: Desired rotation pct output
        :param priority: Mode priority
        :param source: String to name the mode (for visuals only)
        """
        rotation = applyDeadband(rotation_speed, JoystickConst.DEADBAND)
        self._rotation_requests.append(
            AxisRequest(
                value=RobotConst.MAX_SPEED_RADPS * rotation,
                priority=priority.value,
                source=source
            )
        )
    
    def publish_telemetry(self) -> None:
        """
        Publishes all drivetrain-specific data before execute() is run.
        """
        self.publish_value("Drive/GyroAngle (degrees)", self.get_heading().degrees())

        self.publish_value(
            "Drive/FLMotorOutput [-1, 1]",
            self.front_left_motor.getAppliedOutput(),
        )
        self.publish_value(
            "Drive/FRMotorOutput [-1, 1]",
            self.front_right_motor.getAppliedOutput(),
        )

        self.publish_value(
            "Drive/FLEncoderPos (rotations)",
            self.front_left_encoder.getPosition(),
        )
        self.publish_value(
            "Drive/FREncoderPos (rotations)",
            self.front_right_encoder.getPosition(),
        )

        self.publish_value(
            "Drive/FLVelocity (rotation per sec)",
            self.front_left_encoder.getVelocity()
        )
        self.publish_value(
            "Drive/FRVelocity (rotations per sec)",
            self.front_right_encoder.getVelocity()
        )

        self.publish_value("Drive/Chassis/xPose", self.get_pose().X())
        self.publish_value("Drive/Chassis/yPose", self.get_pose().Y())
        self.publish_value("Drive/Chassis/yaw (radians)", self.get_pose().rotation().radians())

        resolved_vx_request = resolve_axis(self._forward_requests)
        resolved_omega_request = resolve_axis(self._rotation_requests)

        # Determine the winning request for forward
        if self._forward_requests:
            best_forward = max(self._forward_requests, key=lambda r: r.priority)
            forward_mode = best_forward.source
            forward_priority = best_forward.priority
        else:
            forward_mode = "None"
            forward_priority = -1

        # Determine the winning request for rotation
        if self._rotation_requests:
            best_rotation = max(self._rotation_requests, key=lambda r: r.priority)
            rotation_mode = best_rotation.source
            rotation_priority = best_rotation.priority
        else:
            rotation_mode = "None"
            rotation_priority = -1

        self.publish_value("Drive/DriveAxis/ResolvedVX (m/s)", resolved_vx_request)
        self.publish_value("Drive/DriveAxis/ResolvedOmega (rad/s)", resolved_omega_request)
        self.publish_value("Drive/DriveAxis/ForwardMode", forward_mode)
        self.publish_value("Drive/DriveAxis/ForwardPriority", forward_priority)
        self.publish_value("Drive/DriveAxis/RotationMode", rotation_mode)
        self.publish_value("Drive/DriveAxis/RotationPriority", rotation_priority)

    def execute(self) -> None:
        """
        Updates the robot's position on the field and drives by the requested speeds.
        This method is automatically called by the AdaptiveComponent scheduler.
        """
        self.odometry.update(
            self.get_heading(),
            self.get_left_distance(),
            self.get_right_distance()
        )

        # Find the requests with the highest priority
        vx = resolve_axis(self._forward_requests)
        omega = resolve_axis(self._rotation_requests)

        chassis = ChassisSpeeds(vx, 0.0, omega)
        wheel_speeds = self.kinematics.toWheelSpeeds(chassis)

        wheel_speeds.desaturate(RobotConst.MAX_SPEED_MPS)

        self.tank_drive_velocity(
            wheel_speeds.left,
            wheel_speeds.right
        )

        self._forward_requests.clear()
        self._rotation_requests.clear()
