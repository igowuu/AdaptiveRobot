import math

from wpimath.units import (
    degrees,
    meters,
    meters_per_second,
    radians,
    radians_per_second
)


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp a value to ensure that it's within the provided limits."""
    return max(lower, min(value, upper))


def radians_to_degrees(
    radians: radians
) -> degrees:
    """Convert a value in radians to degrees."""
    return radians * (180 / math.pi)

def degrees_to_radians(
    degrees: degrees
) -> radians:
    """Convert a value in degrees to radians."""
    return degrees * (math.pi / 180)


def rotations_to_meters(
    motor_rotations: float,
    wheel_diameter: meters,
    motor_to_wheel_ratio: float = 1.0
) -> meters:
    """Convert motor encoder rotations to linear travel distance."""
    wheel_rotations = motor_rotations / motor_to_wheel_ratio
    wheel_circumference = meters(math.pi * wheel_diameter)
    return meters(wheel_rotations * wheel_circumference)

def meters_to_rotations(
    distance: meters,
    wheel_diameter: meters,
    motor_to_wheel_ratio: float = 1.0
) -> float:
    """Convert linear travel distance to motor encoder rotations."""
    wheel_circumference = meters(math.pi * wheel_diameter)
    wheel_rotations = distance / wheel_circumference
    return wheel_rotations * motor_to_wheel_ratio


def mps_to_rps(
    velocity: meters_per_second,
    wheel_diameter: meters,
    motor_to_wheel_ratio: float = 1.0
) -> float:
    """Convert meters per second to motor encoder rotations per second."""
    wheel_circumference = meters(math.pi * wheel_diameter)
    wheel_rps = velocity / wheel_circumference
    return wheel_rps * motor_to_wheel_ratio

def rps_to_mps(
    motor_rps: float,
    wheel_diameter: meters,
    motor_to_wheel_ratio: float = 1.0
) -> meters_per_second:
    """Convert motor rotations per second to velocity in meters per second."""
    wheel_rps = motor_rps / motor_to_wheel_ratio
    wheel_circumference = meters(math.pi * wheel_diameter)
    return meters_per_second(wheel_rps * wheel_circumference)


def rps_to_rad_per_sec(
    motor_rps: float,
    motor_to_arm_ratio: float = 1.0
) -> radians_per_second:
    """Convert motor rotations per second to radians per second."""
    arm_rps = motor_rps / motor_to_arm_ratio
    return radians_per_second(arm_rps * 2.0 * math.pi)

def rad_per_sec_to_rps(
    arm_rad_per_sec: radians_per_second,
    motor_to_arm_ratio: float = 1.0
) -> float:
    """Convert radians per second to motor rotations per second."""
    arm_rps = arm_rad_per_sec / (2.0 * math.pi)
    return arm_rps * motor_to_arm_ratio


def rotations_to_radians(
    motor_rotations: float,
    motor_to_arm_ratio: float = 1.0
) -> radians:
    """Convert motor rotations to radians."""
    arm_rotations = motor_rotations / motor_to_arm_ratio
    return radians(arm_rotations * 2.0 * math.pi)

def radians_to_rotations(
    arm_radians: radians,
    motor_to_arm_ratio: float = 1.0
) -> float:
    """Convert radians to motor rotations."""
    arm_rotations = arm_radians / (2.0 * math.pi)
    return arm_rotations * motor_to_arm_ratio


def degrees_to_rotations(
    arm_degrees: degrees,
    motor_to_arm_ratio: float = 1.0
) -> float:
    """Convert degrees to motor rotations."""
    arm_rotations = arm_degrees / 360.0
    return arm_rotations * motor_to_arm_ratio

def rotations_to_degrees(
    motor_rotations: float,
    motor_to_arm_ratio: float = 1.0
) -> degrees:
    """Convert motor rotations to degrees."""
    arm_rotations = motor_rotations / motor_to_arm_ratio
    return degrees(arm_rotations * 360.0)


def rps_to_rpm (
    rotations_per_second: float
) -> float:
    """Convert a rate in units per second into a rate per minute."""
    return rotations_per_second * 60.0

def rpm_to_rps(
    rotations_per_minute: float
) -> float:
    """Convert a rate in units per minute into a rate per second."""
    return rotations_per_minute / 60.0
