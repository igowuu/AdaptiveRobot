from navx import AHRS

from wpilib import RobotBase
from wpimath.units import degrees
from wpimath.geometry import Rotation2d


class NavXSim:
    """
    NavXSim constructs a simulated NavX gyro that behaves similarly to the actual hardware.
    This is a simple implementation with the commonly used methods within the library. NavXSim 
    directly alters the values of the navx object you pass into it.
    """

    def __init__(self, navx: AHRS) -> None:
        self.gyro = navx
        self.angle_degrees = 0.0
    
    def get_angle(self) -> degrees:
        """Gets the gyro angle in degrees (from -180 to 180)."""
        return self.angle_degrees
    
    def set_angle(self, angle: degrees) -> None:
        """Sets the gyro angle to a value in degrees (from -180 to 180)."""
        self.angle_degrees = angle
        self.gyro.setAngleAdjustment(angle)

    def reset(self) -> None:
        """Resets the gyro to a zero angle."""
        self.angle_degrees = 0.0
        self.gyro.reset()

    def update(self, robot_orientation: float) -> None:
        """Update the gyro angle based on robot rotation."""
        self.angle_degrees = robot_orientation
        if RobotBase.isSimulation():
            self.gyro.setAngleAdjustment(self.angle_degrees)

    def getRotation2d(self) -> Rotation2d:
        """Convert continuous angle to Rotation2d."""
        return Rotation2d.fromDegrees(self.angle_degrees)
