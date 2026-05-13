import math

from components.drivetrain.drivetrain import Drivetrain

from wpimath.units import radians
from wpimath.geometry import Translation2d
from wpimath.controller import PIDController

from adaptive_robot import AsyncAction, BasicPriority


HUB_POSITION = Translation2d(4.03, 4.10)

ANGLE_KP = 3.0
ANGLE_KI = 0.0
ANGLE_KD = 0.6


def wrap_angle(angle: radians) -> radians:
    """
    Wraps an angle in radians to the range from negative pi to pi.
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi


def point_to_hub(drivetrain: Drivetrain) -> AsyncAction:
    """
    Points the drivetrain to the hub using an angular percent PIDController.
    """
    angle_pid_controller = PIDController(ANGLE_KP, ANGLE_KI, ANGLE_KD)

    # Loop until the angle is within 0.1 radians of the target.
    try:
        while True:
            current_angle = drivetrain.io.get_angle().radians()
            target_angle = (HUB_POSITION - drivetrain.get_pose().translation()).angle().radians()
            angle_error = wrap_angle(target_angle - current_angle)

            if abs(angle_error) <= 0.1:
                break

            pid_angular_velocity = angle_pid_controller.calculate(current_angle, target_angle)
            drivetrain.request_angular_velocity(pid_angular_velocity, BasicPriority.AUTO, "auto")

            yield

    finally:
        drivetrain.request_angular_velocity(0.0, BasicPriority.AUTO, "auto")
