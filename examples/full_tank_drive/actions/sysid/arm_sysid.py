from adaptive_robot import AsyncAction, Mechanism, Config, SysIdTest, wait

from components.arm.arm import Arm
from components.arm.arm_io.io_base import ArmIOBase


class ArmSysIDRoutine:
    """
    SysID routine for characterizing arm motor feedforward constants.
    
    Accounts for gravity (static friction, gravity compensation coefficients).
    Automatically locks the arm component to bypass closed-loop control.
    """
    def __init__(self, arm: Arm, arm_io: ArmIOBase) -> None:
        """
        Creates an ArmSysIDRoutine for the given arm.
        """
        self.arm = arm
        self.io = arm_io
        self.config = Config()

        self.arm_motor = Mechanism(
            command_voltage=self.io.set_voltage,
            get_voltage=self.io.get_voltage,
            get_distance=self.io.get_position,
            get_velocity=self.io.get_velocity,
            name="Arm"
        )

        self.tests = SysIdTest.create_all_tests(self.arm_motor, self.config)

    def _create_locked_test(self, test: SysIdTest) -> AsyncAction:
        """
        Wraps a SysIdTest with component locking lifecycle.
        Manually updates IO sensors each iteration since component is locked.

        :param test: The SysIdTest to wrap.
        :return: An AsyncAction that locks, runs test, and unlocks.
        """
        try:
            self.arm.locked = True
            for _ in test.run():
                self.io.update()
                yield
        finally:
            self.arm.locked = False
            self.io.set_voltage(0.0)

    def full_test(self) -> AsyncAction:
        """
        Executes all of the tests sequentially.
        """
        yield from self._create_locked_test(self.tests[0])
        yield from wait(3)
        yield from self._create_locked_test(self.tests[1])
        yield from wait(3)
        yield from self._create_locked_test(self.tests[2])
        yield from wait(3)
        yield from self._create_locked_test(self.tests[3])
