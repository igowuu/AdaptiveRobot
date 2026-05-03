from adaptive_robot import AsyncAction, BasicPriority, wait

from arm import Arm
from arm_constants import ArmConstants


def demo_auto(arm: Arm) -> AsyncAction:
    """
    This auto moves the arm up and then down.
    It doesn't do anything particularly useful, but it demonstrates usage.
    """
    # Request max angle
    while True:
        angle_error = ArmConstants.MAX_ANGLE - arm.io.get_position()
        if angle_error < ArmConstants.ANGLE_TOLERANCE:
            break
        else:
            arm.request_angle(ArmConstants.MAX_ANGLE, BasicPriority.AUTO, "auto")
        yield
    
    yield from wait(0.5)    # Wait 0.5 seconds

    # Request min angle
    while True:
        angle_error = arm.io.get_position() - ArmConstants.MIN_ANGLE
        if angle_error < ArmConstants.ANGLE_TOLERANCE:
            break
        else:
            arm.request_angle(ArmConstants.MIN_ANGLE, BasicPriority.AUTO, "auto")
        yield

    # You can also call non-blocking methods here. For example:
    print("Finished demo_auto!")
