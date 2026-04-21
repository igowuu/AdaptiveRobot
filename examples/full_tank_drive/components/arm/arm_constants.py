import math

from wpimath.units import (
    volts, 
    meters, 
    kilogram_square_meters, 
    radians_per_second
)


class ArmConstants:
    MAX_VOLTAGE: volts = 12.0

    ARM_GEAR_RATIO = 9.18
    ARM_MOI: kilogram_square_meters = 0.1
    ARM_LENGTH: meters = 0.3

    MAX_VELOCITY: radians_per_second = 7.8
    
    MIN_ANGLE = 0.0
    MAX_ANGLE = math.pi / 4


class ArmCAN:
    """
    Holds the CAN ids for each motor used in the component.
    """
    LEFT = 5
    RIGHT = 6


class ArmPID:
    """
    Holds the position PID gains for the arm component.
    """
    KP = 5.0
    KI = 0.0
    KD = 0.8

class ArmFF:
    """
    Holds the position FF gains for the arm component.
    """
    KS = 0.08
    KG = 1.204
    KV = 0.182
    KA = 0.027
