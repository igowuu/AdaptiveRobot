from wpimath.system.plant import DCMotor
from wpimath.units import seconds
from wpilib.simulation import SingleJointedArmSim, DutyCycleEncoderSim
from wpilib import Mechanism2d, SmartDashboard

from components.arm import Arm

from utils.math_utils import degrees_to_rotations
from config.constants import ArmConst


class ArmSim:
    def __init__(self, arm: Arm) -> None:
        self.arm = arm

        self.arm_gearbox = DCMotor.CIM(2)
        
        # Initialize all simulated hardware
        self.arm_sim = SingleJointedArmSim(
            gearbox=self.arm_gearbox,
            gearing=ArmConst.GEAR_RATIO,
            moi=ArmConst.MOI,
            armLength=ArmConst.LENGTH,
            minAngle=ArmConst.MIN_ANGLE,
            maxAngle=ArmConst.MAX_ANGLE,
            simulateGravity=True,
            startingAngle=ArmConst.MIN_ANGLE
        )
        
        self.arm_right_encoder_sim = DutyCycleEncoderSim(self.arm.right_arm_encoder)
        self.arm_right_motor_sim = self.arm.right_arm_motor.getSimCollection()

        self.arm_left_encoder_sim = DutyCycleEncoderSim(self.arm.left_arm_encoder)
        self.arm_left_motor_sim = self.arm.left_arm_motor.getSimCollection()

        # Set up arm GUI
        self.arm_sim_vis = Mechanism2d(ArmConst.SIM_WIDTH, ArmConst.SIM_HEIGHT)

        self.arm_root = self.arm_sim_vis.getRoot(
            name="Arm Root", 
            x=ArmConst.ROOT_X, 
            y=ArmConst.ROOT_Y
        )
        self.arm_ligament = self.arm_root.appendLigament(
            name="Arm", 
            length=ArmConst.ROOT_LENGTH,
            angle=ArmConst.ROOT_ANGLE
        )

        SmartDashboard.putData("Arm Sim", self.arm_sim_vis)

    def execute(self, tm_diff: seconds) -> None:
        """Updates the arm angle based on the simulated motor output voltage."""
        self.arm_sim.setInput(0, self.arm_left_motor_sim.getMotorOutputLeadVoltage())
        
        self.arm_sim.update(tm_diff)

        arm_degrees = self.arm_sim.getAngleDegrees()
        arm_rot = degrees_to_rotations(arm_degrees)

        self.arm_left_encoder_sim.set(arm_rot)
        self.arm_right_encoder_sim.set(arm_rot)
        self.arm_ligament.setAngle(arm_degrees)
