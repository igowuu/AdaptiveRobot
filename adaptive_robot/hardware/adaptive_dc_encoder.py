import math
from typing import Optional

from wpilib import DutyCycleEncoder, Timer

from adaptive_robot.utils.math_utils import rotations_to_radians


class AdaptiveDCEncoder(DutyCycleEncoder):
    """
    Wrapper for a wpilib DutyCycleEncoder to conveniently access values with minimal boilerplate.
    """
    def __init__(self, channel: int) -> None:
        """
        Construct a new DutyCycleEncoder on a specific channel.

        This has a fullRange of 1 and an expectedZero of 0.
        
        :param channel: the RIO port to attach to.
        """
        super().__init__(channel)

        self.prev_time = Timer.getFPGATimestamp()
        self.prev_angle = self.get_position()

    def get_position(self, gear_ratio: Optional[float] = None) -> float:
        """
        Returns the total distance traveled in radians.
        
        :param gear_ratio: Optional gear ratio. Should be used only if the encoder is attached to a motor.
        :return: Total distance traveled in radians.
        """
        rotations = self.get()

        if gear_ratio is None:
            return rotations_to_radians(rotations)
        else:
            return rotations_to_radians(rotations, gear_ratio)
        
    def get_velocity(self) -> float:
        """
        Returns the current velocity in radians per second.
        
        :return: Velocity in radians / sec.
        """
        now = Timer.getFPGATimestamp()
        angle = self.get_position()

        dt = now - self.prev_time
        if dt <= 0:
            return 0.0

        delta = angle - self.prev_angle

        # Unwrap
        if delta > math.pi:
            delta -= 2 * math.pi
        elif delta < -math.pi:
            delta += 2 * math.pi

        self.prev_angle = angle
        self.prev_time = now

        return delta / dt
