import math

from wpimath.units import (
    kilograms, meters, meters_per_second, meters_per_second_squared, 
    percent, radians, radians_per_second, volts, kilogram_square_meters
)

# Percent values are normalized: 1.0 == 100%

class RobotConst:
    MASS: kilograms = 100.0
    WHEEL_DIAMETER: meters = 0.1524
    MAX_SPEED_MPS: meters_per_second = 4.23
    MAX_SPEED_RADPS: radians_per_second = 12.3
    MAX_ACCEL_MPS_SQUARED: meters_per_second_squared = 2.20
    TRACK_WIDTH: meters = 0.6  # Distance between left and right wheels
    MAX_VOLTAGE: volts = 12.0 # Maximum possible voltage for all bot components


# Joystick constants
class JoystickConst:
    DEADBAND: percent = 0.15


# Drivetrain constants
class DrivetrainConst:
    GEAR_RATIO = 10.71  # Motor rotations per wheel rotation
    MOI: kilogram_square_meters = 0.1  # Moment of intertia of drivetrain (estimate)
    SLEW_FORWARD: meters_per_second = 3.0
    SLEW_ROTATION: radians_per_second = 8.7


# Drivetrain PID
class DrivetrainPID:
    # Left gearbox PID (input in m/s, output in volts - estimates)
    LEFT_KP = 0.01
    LEFT_KI = 0.0
    LEFT_KD = 0.0

    # Right gearbox PID (input in m/s, output in volts - estimates)
    RIGHT_KP = 0.01
    RIGHT_KI = 0.0
    RIGHT_KD = 0.0

    B = 2.0
    ZETA = 0.7


# Drivetrain feedforward
class DrivetrainFF:
    # Left gearbox feedforward (estimates)
    LEFT_KS = 0.50  # volts, static gain
    LEFT_KV = 2.60  # volts per meter per second
    LEFT_KA = 0.05  # volts per meter per second^2

    # Right gearbox feedforward (estimates)
    RIGHT_KS = 0.50  # volts, static gain
    RIGHT_KV = 2.60  # volts per meter per second
    RIGHT_KA = 0.05  # volts per meter per second^2


# Arm constants
class ArmConst:
    GEAR_RATIO = 30.0  # motor rotations per arm rotation
    MOI: kilogram_square_meters = 0.1  # Moment of inertia of arm (estimate)
    LENGTH: meters = 0.2  # Arm length
    MIN_ANGLE: radians = 0.0
    MAX_ANGLE: radians = math.pi / 2.0
    APPLIED_PCT_OUT: percent = 1.0

    # Simulation constants (unitless, all in terms of the GUI)
    SIM_HEIGHT = 50.0
    SIM_WIDTH = 20.0
    ROOT_LENGTH = 10.0
    ROOT_ANGLE = 0.0
    ROOT_X = 10.0
    ROOT_Y = 1.0


# Intake constants
class IntakeConst:
    APPLIED_PCT_OUT: percent = 1.0