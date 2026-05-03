import math
from dataclasses import dataclass
from typing import Optional
from components.shooter.shooter_constants import ShooterConstants

from wpimath.geometry import Translation2d
from wpimath.units import meters_per_second, radians_per_second


@dataclass
class ShooterCalculationResult:
    """
    Represents the possible results of a shooter velocity calculation.
    """
    success: bool
    velocity: Optional[meters_per_second] = None
    error: Optional[str] = None


def calculate_optimal_velocity(
    target_translation: Translation2d,
    gravity: float = 9.81,
) -> ShooterCalculationResult:
    """
    Calculates the optimal shooter velocity to reach a target translation.

    Uses projectile motion physics with a fixed hood angle to find the velocity
    required to hit a target at (translation x, translation y) relative to the shooter.

    :param target_translation: The position of the target relative to the shooter 
    (x = horizontal distance, y = vertical height difference)  
    :param gravity_accel: Gravitational acceleration in meters per second squared.

    :returns: A ShooterCalculationResult object.
    """
    target_x = target_translation.X()
    target_y = target_translation.Y()

    theta = ShooterConstants.SHOOTER_HOOD_ANGLE
    y0 = ShooterConstants.SHOOTER_HEIGHT
    max_velocity = ShooterConstants.MAX_VELOCITY
    flywheel_radius = ShooterConstants.FLYWHEEL_RADIUS

    max_linear_velocity = max_velocity * flywheel_radius

    if target_x <= 0:
        return ShooterCalculationResult(
            success=False,
            error="Target distance must be positive"
        )

    # Calculate required velocity via projectile motion
    cos_theta = math.cos(theta)
    tan_theta = math.tan(theta)

    denominator_scalar = y0 + target_x * tan_theta - target_y
    denominator = 2 * (cos_theta ** 2) * denominator_scalar

    if denominator <= 0:
        return ShooterCalculationResult(
            success=False,
            error="Target is unreachable (so above possible trajectory)"
        )

    numerator = gravity * (target_x ** 2)

    velocity_squared = numerator / denominator
    if velocity_squared < 0:
        return ShooterCalculationResult(
            success=False,
            error="Target calculation resulted in negative vel squared"
        )

    velocity = math.sqrt(velocity_squared)

    if velocity > max_linear_velocity:
        return ShooterCalculationResult(
            success=False,
            error=f"Required velocity {velocity:.2f} m/s is above maximum {max_linear_velocity:.2f} m/s"
        )

    return ShooterCalculationResult(
        success=True,
        velocity=velocity
    )


def velocity_to_motor_output(velocity: meters_per_second) -> radians_per_second:
    """
    Converts linear velocity to motor output.

    :param velocity: Linear velocity at the flywheel surface (m/s)

    :returns Angular velocity needed (rad/s)
    """
    flywheel_radius = ShooterConstants.FLYWHEEL_RADIUS
    return velocity / flywheel_radius
