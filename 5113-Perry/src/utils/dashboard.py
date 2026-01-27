from typing import TYPE_CHECKING

from rev import SparkMax

from ntcore import NetworkTableInstance
from wpilib import RobotController, DriverStation

from components.drivetrain import Drivetrain

from adaptive_robot.adaptive_component import AdaptiveComponent

if TYPE_CHECKING:
    from robot import Perry


class Dashboard(AdaptiveComponent):
    """
    Handles publishing robot info that isn't tied to a specific subsystem to NetworkTables.
    """

    def __init__(
        self,
        robot: "Perry",
        drivetrain: Drivetrain
    ) -> None:
        super().__init__(robot)

        self.drivetrain = drivetrain

        self.nt = NetworkTableInstance.getDefault().getTable("Dashboard")
        self._round_digits = 5

    def _put_general(self) -> None:
        """
        Posts general robot information to dashboard.
        """
        self.nt.putNumber(
            "General/Battery Voltage (volts)",
            RobotController.getBatteryVoltage(),
        )
        self.nt.putBoolean(
            "General/Enabled", 
            DriverStation.isEnabled()
        )

    def _put_sparkmax_faults(self, name: str, motor: SparkMax) -> None:
        """
        Post all possible SparkMax faults to dashboard under Safety, given the
        motor name and object.
        """
        motor_faults = motor.getStickyFaults()
        
        self.nt.putBoolean(f"Safety/{name}/Can", motor_faults.can)
        self.nt.putBoolean(f"Safety/{name}/Firmware", motor_faults.firmware)
        self.nt.putBoolean(f"Safety/{name}/GateDriver", motor_faults.gateDriver)
        self.nt.putBoolean(f"Safety/{name}/MotorType", motor_faults.motorType)
        self.nt.putBoolean(f"Safety/{name}/Sensor", motor_faults.sensor)
        self.nt.putBoolean(f"Safety/{name}/Temperature", motor_faults.temperature)
        self.nt.putBoolean(f"Safety/{name}/Other", motor_faults.other)

    def _put_motor_faults(self) -> None:
        """
        Posts all motor faults (if any) to dashboard.
        """
        self.nt.putBoolean("Safety/FL/IsFLFault", self.drivetrain.front_left_motor.hasStickyFault())
        self.nt.putBoolean("Safety/FR/IsFRFault", self.drivetrain.front_right_motor.hasStickyFault())

        self._put_sparkmax_faults("FL", self.drivetrain.front_left_motor)
        self._put_sparkmax_faults("FR", self.drivetrain.front_right_motor)

    def _put_safety(self) -> None:
        """
        Posts all information regarding robot safety and maintenence to dashboard.
        If something with the hardware goes wrong, it will show here.
        """
        self.nt.putBoolean(
            "Safety/Brownout",
            RobotController.isBrownedOut()
        )
        self._put_motor_faults()

    def execute(self) -> None:
        """Update all dashboard values."""
        self._put_general()
        self._put_safety()
