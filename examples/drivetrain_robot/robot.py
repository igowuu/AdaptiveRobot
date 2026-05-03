from adaptive_robot import AdaptiveRobot

from drivetrain import Drivetrain
from drivetrain_controller import DrivetrainController

from wpilib import Joystick


class MyRobot(AdaptiveRobot):
    def __init__(self) -> None:
        # Calls the parent constructor to initialize the AdaptiveRobot module.
        super().__init__()
    
    def onRobotInit(self) -> None:
        # Robot-wide initialization goes in the onRobotInit hook.
        # All subclasses of interfaces (AdaptiveRobot, TunablePublishable,
        # TelemetryPublishable, Schedulable) are automatically registered into the scheduler
        # if declared here.
        self.controller = Joystick(0)

        # Both of these inherit from interfaces, so they are automatically registered into the scheduler.
        self.drivetrain = Drivetrain()
        self.drivetrain_controller = DrivetrainController(self.drivetrain, self.controller)
