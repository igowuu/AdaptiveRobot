import choreo
import wpilib

from wpimath.controller import LTVUnicycleController

from components.drivetrain.drivetrain_constants import DriveConstants
from components.drivetrain.drivetrain import Drivetrain

from adaptive_robot import AsyncAction, BasicPriority


def follow_trajectory(drivetrain: Drivetrain, trajectory_name: str) -> AsyncAction:
    """
    Follows a Choreo trajectory using LTVUnicycleController.

    :param drivetrain: The drivetrain component
    :param trajectory_name: Name of the Choreo trajectory
    """
    trajectory = choreo.load_differential_trajectory(trajectory_name)
    samples = trajectory.get_samples()

    if not samples:
        return

    controller = LTVUnicycleController(
        dt=0.02,
        maxVelocity=DriveConstants.MAX_LINEAR_SPEED
    )

    timer = wpilib.Timer()
    timer.restart()
    sample_index = 0
    total_time = samples[-1].timestamp

    try:
        while timer.get() < total_time:
            t = timer.get()

            # Typically, trajectory.sample_at(t) should be used.
            # As of 4-19-2026, an internal error exists within choreo that prevents correct
            # use of this method; therefore, sampling is manual for this Action.
            while sample_index + 1 < len(samples) and samples[sample_index + 1].timestamp <= t:
                sample_index += 1

            sample = samples[sample_index]

            desired_pose = sample.get_pose()
            desired_chassis = sample.get_chassis_speeds()
            current_pose = drivetrain.get_pose()

            chassis_speeds = controller.calculate(
                currentPose=current_pose, 
                poseRef=desired_pose,
                linearVelocityRef=desired_chassis.vx,
                angularVelocityRef=desired_chassis.omega
            )

            drivetrain.request_linear_velocity(chassis_speeds.vx, BasicPriority.AUTO, "auto")
            drivetrain.request_angular_velocity(chassis_speeds.omega, BasicPriority.AUTO, "auto")

            yield
    
    finally:
        timer.stop()
        drivetrain.request_linear_velocity(0.0, BasicPriority.AUTO, "auto")
        drivetrain.request_angular_velocity(0.0, BasicPriority.AUTO, "auto")
