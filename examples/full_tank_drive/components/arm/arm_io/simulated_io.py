from wpilib import Mechanism2d, SmartDashboard, Timer
from wpilib.simulation import SingleJointedArmSim

from wpimath.units import volts, radians, radians_per_second
from wpimath.system.plant import DCMotor

from components.arm.arm_io.io_base import ArmIOBase
from components.arm.arm_constants import ArmConstants

from adaptive_robot.utils.math_utils import clamp


class SimulatedArmIO(ArmIOBase):
    """
    IO that allows the robot to run without hardware.
    """
    def __init__(self) -> None:
        self._arm_sim = SingleJointedArmSim(
            gearbox=DCMotor.NEO(2),
            gearing=ArmConstants.ARM_GEAR_RATIO,
            moi=ArmConstants.ARM_MOI,
            armLength=ArmConstants.ARM_LENGTH,
            minAngle=ArmConstants.MIN_ANGLE,
            maxAngle=ArmConstants.MAX_ANGLE,
            simulateGravity=True,
            startingAngle=ArmConstants.MIN_ANGLE
        )

        arm_gui_width = 20
        arm_gui_length = 50
        arm_visiblity = Mechanism2d(arm_gui_width, arm_gui_length)

        root_name = "ArmRoot"
        root_x = 10
        root_y = 1
        self.arm_root = arm_visiblity.getRoot(root_name, root_x, root_y)

        ligament_name = "Arm"
        ligament_length = 10
        ligament_angle = ArmConstants.MIN_ANGLE

        self._arm_ligament = self.arm_root.appendLigament(
            ligament_name, ligament_length, ligament_angle
        )

        SmartDashboard.putData("Arm Sim", arm_visiblity)

        self._desired_voltage = 0.0

        self._previous_timestamp = Timer.getFPGATimestamp()
    
    def get_position(self) -> radians:
        return self._arm_sim.getAngle()
    
    def get_velocity(self) -> radians_per_second:
        return self._arm_sim.getVelocity()

    def get_voltage(self) -> volts:
        return self._desired_voltage

    def set_voltage(self, voltage: volts) -> None:
        self._desired_voltage = voltage

    def update(self) -> None:
        self._arm_sim.setInputVoltage(
            clamp(self._desired_voltage, -ArmConstants.MAX_VOLTAGE, ArmConstants.MAX_VOLTAGE)
        )

        new_timestamp = Timer.getFPGATimestamp()
        self._arm_sim.update(new_timestamp - self._previous_timestamp)
        self._previous_timestamp = new_timestamp
        self._arm_ligament.setAngle(self._arm_sim.getAngleDegrees())
