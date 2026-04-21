from adaptive_robot import AsyncAction

from components.drivetrain.drivetrain import Drivetrain

from actions.trajectories.follow_trajectory import follow_trajectory


def taxi_drive(drivetrain: Drivetrain) -> AsyncAction:
    """
    Drives the robot a few meters forward.
    """
    yield from follow_trajectory(drivetrain, "taxi_drive")
