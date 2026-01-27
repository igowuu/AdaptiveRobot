from typing import TYPE_CHECKING

from pyfrc.physics.core import PhysicsInterface

from sim.drivetrain_sim import DrivetrainSim
from sim.arm_sim import ArmSim

if TYPE_CHECKING:
    from robot import Perry


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "Perry"):
        self.physics_controller = physics_controller
        self.drivetrain = robot.drivetrain
        self.arm = robot.arm

        self.drivetrain_sim = DrivetrainSim(physics_controller, self.drivetrain)
        self.arm_sim = ArmSim(self.arm)

    def update_sim(self, now: float, tm_diff: float) -> None:
        self.drivetrain_sim.execute(tm_diff)
        self.arm_sim.execute(tm_diff)
