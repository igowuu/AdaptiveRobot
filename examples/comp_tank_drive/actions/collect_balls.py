from adaptive_robot import AsyncAction

from actions.follow_trajectory import follow_trajectory
from components.drivetrain.drivetrain import Drivetrain


def collect_balls(drivetrain: Drivetrain) -> AsyncAction:
    """
    Follows the `collect balls` trajectory.
    """
    yield from follow_trajectory(drivetrain, "collect_balls")
