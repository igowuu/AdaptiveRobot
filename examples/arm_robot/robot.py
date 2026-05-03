from adaptive_robot import AdaptiveRobot

from arm_io.real_io import RealArmIO
from arm_io.simulated_io import SimulatedArmIO
from arm import Arm
from arm_controller import ArmController

from wpilib import Joystick

from autonomous.demo_auto import demo_auto


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

        # If in simulation mode, use the simulated IO; else, use the real IO.
        if self.isSimulation():
            self.arm_io = SimulatedArmIO()
        else:
            self.arm_io = RealArmIO()

        # Both of these inherit from interfaces, so they are automatically registered into the scheduler.
        self.arm = Arm(self.arm_io)
        self.arm_controller = ArmController(self.arm, self.controller)

    def onAutonomousInit(self) -> None:
        # Called once when autonomous mode is entered.
        self.schedule_action(demo_auto(self.arm), "demo_arm")
