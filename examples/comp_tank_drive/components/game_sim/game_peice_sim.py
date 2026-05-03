from wpilib import RobotBase, RobotController

from wpimath.geometry import Translation3d
from wpimath.units import meters_per_second

from components.drivetrain.drivetrain import Drivetrain

from components.intake_arm.intake_arm_io.io_base import IntakeArmIOBase
from components.intake_arm.intake_arm_constants import IntakeArmConstants
from components.intake.intake_constants import IntakeConstants

from components.shooter.shooter_io.io_base import ShooterIOBase
from components.shooter.shooter_constants import ShooterConstants

from fuel_sim.fuel_sim import FuelSim

from constants import RobotConst

from adaptive_robot import Schedulable


class GamePieceSim(Schedulable):
    """
    Simulates fuel, hubs, collisions, and points within AdvantageScope.
    """
    def __init__(
        self, 
        drivetrain: Drivetrain, 
        intake_arm_io: IntakeArmIOBase, 
        shooter_io: ShooterIOBase
    ) -> None:
        self.drivetrain = drivetrain
        self.intake_arm_io = intake_arm_io
        self.shooter_io = shooter_io

        self.fuel_sim = FuelSim()

        self.spawn_fuel_line()

        self.fuel_sim.register_robot(
            RobotConst.ROBOT_WIDTH,
            RobotConst.ROBOT_LENGTH,
            RobotConst.BUMPER_HEIGHT,
            lambda: self.drivetrain.get_pose(),
            lambda: self.drivetrain.get_chassis()
        )

        self.fuel_sim.enable_air_resistance()

        intake_length_half = IntakeConstants.INTAKE_LENGTH / 2
        intake_width = IntakeConstants.INTAKE_WIDTH
        intake_front = RobotConst.ROBOT_LENGTH / 2

        min_angle = IntakeArmConstants.MIN_ANGLE

        # The amount of radians where the intake will still pick up balls of the ground,
        # even though the intake arm isn't at its minimum angle.
        min_angle_tol = IntakeArmConstants.MIN_ANGLE_TOLERANCE

        self.fuel_sim.register_intake(
            x_min=intake_front, 
            x_max=intake_front + intake_width, 
            y_min=-intake_length_half, 
            y_max=intake_length_half,
            able_to_intake=lambda: self.intake_arm_io.get_position() <= min_angle + min_angle_tol
        )

        self.fuel_sim.log_fuels()

        self.fuel_sim.set_subticks(5)
        self.fuel_sim.set_logging_frequency(300)

        self.fuel_sim.start()
    
    def spawn_fuel_line(self) -> None:
        """
        Spawns a single line of fuel in the middle of the field.
        """
        self.fuel_sim.spawn_fuel(Translation3d(8.0, 4.0, 0.05), Translation3d(0, 0, 0))
        self.fuel_sim.spawn_fuel(Translation3d(8.2, 4.0, 0.05), Translation3d(0, 0, 0))
        self.fuel_sim.spawn_fuel(Translation3d(8.4, 4.0, 0.05), Translation3d(0, 0, 0))
        self.fuel_sim.spawn_fuel(Translation3d(8.6, 4.0, 0.05), Translation3d(0, 0, 0))
        self.fuel_sim.spawn_fuel(Translation3d(8.8, 4.0, 0.05), Translation3d(0, 0, 0))
        self.fuel_sim.spawn_fuel(Translation3d(9.0, 4.0, 0.05), Translation3d(0, 0, 0))
    
    def clear_all_fuel(self) -> None:
        """
        Clears all fuel currently in the field.
        """
        self.fuel_sim.clear_fuel()
    
    def _get_ball_velocity(self) -> meters_per_second:
        """
        Calculates the ball velocity in m/s based on current shooter voltage.
        
        :return: Ball velocity in m/s
        """
        percent = self.shooter_io.get_voltage() / RobotController.getBatteryVoltage()
        motor_velocity = percent * ShooterConstants.MAX_VELOCITY
        ball_velocity = motor_velocity * ShooterConstants.FLYWHEEL_RADIUS
        
        return ball_velocity

    def shoot_fuel(self) -> None:
        """
        Shoots a single fuel object with the currently commanded velocity to the shooter.
        """
        self.fuel_sim.launch_fuel(
            launch_velocity=self._get_ball_velocity(),
            hood_angle=ShooterConstants.SHOOTER_HOOD_ANGLE,
            shooter_yaw=0.0,
            launch_height=ShooterConstants.SHOOTER_HEIGHT
        )

    def execute(self) -> None:
        """
        Updates the state of the fuel sim each iteration in simulation mode.
        """
        if RobotBase.isSimulation():
            self.fuel_sim.update_sim()
