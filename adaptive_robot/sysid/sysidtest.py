from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable

from wpilib.sysid import SysIdRoutineLog, State
from wpilib import Timer, DriverStation
from wpimath.units import seconds, volts

from adaptive_robot.utils.math_utils import clamp


volts_per_second = float


@dataclass
class Config:
    rampRate: volts_per_second = 1.0
    stepVoltage: volts = 7.0
    timeout: seconds = 10.0
    maximum_volts: volts = 12.0


class Mechanism:
    """
    Represents a single mechanism. A mechanism is (but is not limited to) a
    singular motor or a whole subsystem.
    """
    def __init__(
        self,
        command_voltage: Callable[[float], None],
        get_voltage: Callable[[], float],
        get_distance: Callable[[], float],
        get_velocity: Callable[[], float],
        name: str
    ) -> None:
        """
        Creates a Mechanism object, storing lambda functions for each of the required elements.
        
        :param command_voltage: Callable that commands voltage to the mechanism.
        :param get_voltage: Callable that returns the measured voltage of the mechanism.
        :param get_distance: Callable that returns the measured linear distance traveled of the mechanism.
        :param get_velocity: Callable that returns the measured linear velocity of the mechanism.
        :param name: The name assigned to the mechanism.
        """
        self.command_voltage = command_voltage
        self.get_voltage = get_voltage
        self.get_distance = get_distance
        self.get_velocity = get_velocity
        self.name = name

    def log(self, logger: SysIdRoutineLog) -> None:
        log_motor = logger.motor(self.name)
        log_motor.voltage(self.get_voltage())
        log_motor.position(self.get_distance())
        log_motor.velocity(self.get_velocity())


class Direction(Enum):
    """
    Holds both types of movement directions for a test.
    """
    kForward = 1
    kReverse = -1


class SysIdTestType(Enum):
    """
    Holds both types of supported test types.
    """
    QUASISTATIC = auto()
    DYNAMIC = auto()


class SysIdTest:
    """
    Represents a full SysIdTest that can be run as an async action to tune FeedForward constants.  
    Unless advanced usage is needed, users should call the static create_all_tests_async() method
    to get a tuple containing all of the necessary test coroutines for a mechanism.
    """
    def __init__(
        self, 
        direction: Direction, 
        config: Config,
        mechanism: Mechanism,
        test_type: SysIdTestType,
        logger: SysIdRoutineLog
    ) -> None:
        """
        Creates a single test coroutine generator. Tests should only attach to one subsystem,
        or one part of a subsystem at a time.
        """
        self.timer = Timer()
        self.direction = direction
        self.config = config
        self.mechanism = mechanism
        self.test_type = test_type
        self.logger = logger

        if self.test_type == SysIdTestType.QUASISTATIC:
            self._state = (
                State.kQuasistaticForward
                if direction == Direction.kForward
                else State.kQuasistaticReverse
            )
        else:
            self._state = (
                State.kDynamicForward
                if direction == Direction.kForward
                else State.kDynamicReverse
            )

        self.running = False

    def _stop(self) -> None:
        """
        Immediately stops all test operations and stops all mechanism movement.
        """
        if not self.running:
            return

        self.mechanism.command_voltage(0.0)
        self.logger.recordState(State.kNone)

        self.timer.stop()
        self.timer.reset()

        self.running = False

    def run(self):
        """
        Generator routine that runs the SysID test.
        Yields control to the scheduler on each iteration.
        """
        if self.running:
            return
        
        self.logger.recordState(State.kNone)
        self.timer.reset()
        self.timer.start()
        self.running = True

        while self.running and self.timer.get() <= self.config.timeout and DriverStation.isEnabled():
            if self.test_type == SysIdTestType.QUASISTATIC:
                output_volts = self.direction.value * self.timer.get() * self.config.rampRate
            else:
                output_volts = self.direction.value * self.config.stepVoltage

            output_volts = clamp(output_volts, -self.config.maximum_volts, self.config.maximum_volts)

            self.mechanism.command_voltage(output_volts)
            self.logger.recordState(self._state)
            self.mechanism.log(self.logger)

            yield

        self._stop()

    def is_running(self) -> bool:
        """
        Returns True if the test is currently running.
        """
        return self.running

    @staticmethod
    def create_all_tests(
        mechanism: Mechanism,
        config: Config
    ) -> tuple["SysIdTest", "SysIdTest", "SysIdTest", "SysIdTest"]:
        """
        Factory method that creates all standard SysID test instances
        (quasistatic + dynamic, forward + reverse).
        
        Returns a tuple: (qs_forward, qs_reverse, dyn_forward, dyn_reverse)
        """
        logger = SysIdRoutineLog(mechanism.name)
        return (
            SysIdTest(
                direction=Direction.kForward, 
                config=config, 
                mechanism=mechanism, 
                test_type=SysIdTestType.QUASISTATIC, 
                logger=logger
            ),
            SysIdTest(
                direction=Direction.kReverse, 
                config=config, 
                mechanism=mechanism, 
                test_type=SysIdTestType.QUASISTATIC, 
                logger=logger
            ),
            SysIdTest(
                direction=Direction.kForward, 
                config=config, 
                mechanism=mechanism, 
                test_type=SysIdTestType.DYNAMIC, 
                logger=logger
            ),
            SysIdTest(
                direction=Direction.kReverse, 
                config=config, 
                mechanism=mechanism, 
                test_type=SysIdTestType.DYNAMIC, 
                logger=logger
            )
        )
