from adaptive_robot import AsyncAction, Mechanism, Config, SysIdTest, wait

from components.drivetrain.drivetrain import Drivetrain
from components.drivetrain.drivetrain_io.io_base import DrivetrainIOBase


class LeftDrivetrainSysIDRoutine:
    """
    SysID routine for characterizing left drivetrain feedforward constants.
    
    Accounts for gravity (static friction, gravity compensation coefficients).
    Automatically locks the drivetrain component to bypass closed-loop control.
    """
    def __init__(self, drivetrain: Drivetrain, drivetrain_io: DrivetrainIOBase) -> None:
        """
        Creates an ArmSysIDRoutine for the given drivetrain.
        """
        self.drivetrain = drivetrain
        self.io = drivetrain_io
        self.config = Config()

        self.arm_motor = Mechanism(
            command_voltage=self.io.set_left_voltage,
            get_voltage=self.io.get_left_voltage,
            get_distance=self.io.get_left_distance,
            get_velocity=self.io.get_left_velocity,
            name="Left drivetrain"
        )

        self.tests = SysIdTest.create_all_tests(self.arm_motor, self.config)

    def _create_locked_test(self, test: SysIdTest) -> AsyncAction:
        """
        Wraps a SysIdTest with component locking lifecycle.
        Manually updates IO sensors each iteration since component is locked.
        """
        try:
            self.drivetrain.locked = True
            for _ in test.run():
                self.io.update()
                yield
        finally:
            self.drivetrain.locked = False
            self.io.set_left_voltage(0.0)

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


class RightDrivetrainSysIDRoutine:
    """
    SysID routine for characterizing right drivetrain feedforward constants.
    
    Accounts for gravity (static friction, gravity compensation coefficients).
    Automatically locks the drivetrain component to bypass closed-loop control.
    """
    def __init__(self, drivetrain: Drivetrain, drivetrain_io: DrivetrainIOBase) -> None:
        """
        Creates an ArmSysIDRoutine for the given drivetrain.
        """
        self.drivetrain = drivetrain
        self.io = drivetrain_io
        self.config = Config()

        self.drivetrain_motor = Mechanism(
            command_voltage=self.io.set_right_voltage,
            get_voltage=self.io.get_right_voltage,
            get_distance=self.io.get_right_distance,
            get_velocity=self.io.get_right_velocity,
            name="Right drivetrain"
        )

        self.tests = SysIdTest.create_all_tests(self.drivetrain_motor, self.config)

    def _create_locked_test(self, test: SysIdTest) -> AsyncAction:
        """
        Wraps a SysIdTest with component locking lifecycle.
        Manually updates IO sensors each iteration since component is locked.
        """
        try:
            self.drivetrain.locked = True
            for _ in test.run():
                self.io.update()
                yield
        finally:
            self.drivetrain.locked = False
            self.io.set_right_voltage(0.0)

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
