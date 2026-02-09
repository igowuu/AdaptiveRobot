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
    gear_ratio: float = 1.0
) -> meters:
    """Convert motor encoder rotations to linear travel distance."""
    wheel_rotations = motor_rotations / gear_ratio
    wheel_circumference = meters(math.pi * wheel_diameter)
    return meters(wheel_rotations * wheel_circumference)

def meters_to_rotations(
    distance: meters,
    wheel_diameter: meters,
    gear_ratio: float = 1.0
) -> float:
    """Convert linear travel distance to motor encoder rotations."""
    wheel_circumference = meters(math.pi * wheel_diameter)
    wheel_rotations = distance / wheel_circumference
    return wheel_rotations * gear_ratio


def mps_to_rps(
    velocity: meters_per_second,
    wheel_diameter: meters,
    gear_ratio: float = 1.0
) -> float:
    """Convert meters per second to motor encoder rotations per second."""
    wheel_circumference = meters(math.pi * wheel_diameter)
    wheel_rps = velocity / wheel_circumference
    return wheel_rps * gear_ratio

def rps_to_mps(
    motor_rps: float,
    wheel_diameter: meters,
    gear_ratio: float = 1.0
) -> meters_per_second:
    """Convert motor rotations per second to velocity in meters per second."""
    wheel_rps = motor_rps / gear_ratio
    wheel_circumference = meters(math.pi * wheel_diameter)
    return meters_per_second(wheel_rps * wheel_circumference)


def rps_to_radps(
    motor_rps: float,
    gear_ratio: float = 1.0
) -> radians_per_second:
    """Convert motor rotations per second to radians per second."""
    motor_rps = motor_rps / gear_ratio
    return radians_per_second(motor_rps * 2.0 * math.pi)

def radps_to_rps(
    motor_radps: radians_per_second,
    gear_ratio: float = 1.0
) -> float:
    """Convert radians per second to motor rotations per second."""
    motor_rps = motor_radps / (2.0 * math.pi)
    return motor_rps * gear_ratio


def rotations_to_radians(
    motor_rotations: float,
    gear_ratio: float = 1.0
) -> radians:
    """Convert motor rotations to radians."""
    motor_rotations = motor_rotations / gear_ratio
    return radians(motor_rotations * 2.0 * math.pi)

def radians_to_rotations(
    motor_radians: radians,
    gear_ratio: float = 1.0
) -> float:
    """Convert radians to motor rotations."""
    motor_rotations = motor_radians / (2.0 * math.pi)
    return motor_rotations * gear_ratio


def degrees_to_rotations(
    degrees: degrees,
    gear_ratio: float = 1.0
) -> float:
    """Convert degrees to motor rotations."""
    arm_rotations = degrees / 360.0
    return arm_rotations * gear_ratio

def rotations_to_degrees(
    motor_rotations: float,
    gear_ratio: float = 1.0
) -> degrees:
    """Convert motor rotations to degrees."""
    arm_rotations = motor_rotations / gear_ratio
    return degrees(arm_rotations * 360.0)


def normalized_rotations_to_radians(
    normalized_rotations: float,
    gear_ratio: float = 1.0
) -> radians:
    """
    Convert normalized motor rotations from [-1, 1] to a value in radians.
    """
    return radians(normalized_rotations * gear_ratio * 2.0 * math.pi)

def radians_to_normalized_rotations(
    motor_radians: float,
    gear_ratio: float = 1.0
) -> radians:
    """
    Convert a value in radians to normalized motor rotations from [-1, 1]
    """
    return motor_radians / (gear_ratio * 2.0 * math.pi)


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
