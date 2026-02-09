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
    rampRate: volts_per_second = 0.5
    stepVoltage: volts = 8.0
    timeout: seconds = 10.0
    maximum_volts: volts = 12.0


class Mechanism:
    def __init__(
        self,
        command_voltage: Callable[[float], None],
        get_voltage: Callable[[], float],
        get_position: Callable[[], float],
        get_velocity: Callable[[], float],
        name: str
    ) -> None:
        """
        Creates a Mechanism object, storing lambda functions for each of the required elements.
        
        :param command_voltage: Callable that commands voltage to the mechanism.
        :param get_voltage: Callable that returns the measured voltage of the mechanism.
        :param get_position: Callable that returns the measured linear position of the mechanism.
        :param get_velocity: Callable that returns the measured linear velocity of the mechanism.
        :param name: The name assigned to the mechanism.
        """
        self.command_voltage = command_voltage
        self.get_voltage = get_voltage
        self.get_position = get_position
        self.get_velocity = get_velocity
        self.name = name

    def log(self, logger: SysIdRoutineLog) -> None:
        log_motor = logger.motor(self.name)
        log_motor.voltage(self.get_voltage())
        log_motor.position(self.get_position())
        log_motor.velocity(self.get_velocity())


class Direction(Enum):
    """
    Holds both types of movement directions for a test.
    """
    kForward = 1
    kReverse = -1


class TestType(Enum):
    """
    Holds both types of supported test types.
    """
    QUASISTATIC = auto()
    DYNAMIC = auto()


class SysIdTest:
    def __init__(
        self, 
        direction: Direction, 
        config: Config,
        mechanism: Mechanism,
        test_type: TestType,
        logger: SysIdRoutineLog
    ) -> None:
        """
        Creates a single test, which must be manually executed and controlled by the user.
        Tests should only attatch to one subsystem, or one part of a subsystem at a time.

        start() - Method used to start the test. This should be called before step() and stop().
        step() - Method used to continue running the test. It should be called each iteration.
        stop() - Immediately commands zero volts to the mechanism and records a 'kNone' state.
        """
        self.timer = Timer()

        self.direction = direction
        self.config = config
        self.mechanism = mechanism
        self.test_type = test_type
        self.logger = logger

        if self.test_type == TestType.QUASISTATIC:
            self.state = (
                State.kQuasistaticForward
                if direction == Direction.kForward
                else State.kQuasistaticReverse
            )
        else:
            self.state = (
                State.kDynamicForward
                if direction == Direction.kForward
                else State.kDynamicReverse
            )

        self.running = False

    def start(self) -> None:
        """
        Starts the timer for testing.
        """
        if self.running:
            return

        self.logger.recordState(State.kNone)

        self.timer.reset()
        self.timer.start()
        self.running = True
    
    def step(self) -> None:
        """
        Advances the test each iteration by calculating the desired volts, executing and logging.
        """
        if not self.running:
            raise AssertionError("Error: SysID step() attempted to run before calling start(). Exited.")

        if self.timer.get() > self.config.timeout:
            self.stop()
            return

        if not DriverStation.isEnabled():
            self.stop()
            return

        if self.test_type == TestType.QUASISTATIC:
            output_volts = self.direction.value * self.timer.get() * self.config.rampRate
        else: # TestType.DYNAMIC
            output_volts = self.direction.value * self.config.stepVoltage

        output_volts = clamp(output_volts, -self.config.maximum_volts, self.config.maximum_volts)

        self.mechanism.command_voltage(output_volts)

        self.logger.recordState(self.state)
        self.mechanism.log(self.logger)

    def stop(self) -> None:
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

    def is_running(self) -> bool:
        """
        Returns True if the test is currently running.
        """
        return self.running


class SysIdRoutine:
    """
    Holds and generates all possible tests, given a mechanism and config.
    """
    def __init__(
        self,
        mechanism: Mechanism,
        config: Config,
        name: str
    ) -> None:
        """
        Creates a SysId routine for a single mechanism.
        
        :param mechanism: Mechanism object with callables to move and obtain values from it.
        :param config: Config object with test-specific info.
        :param name: Name for the SysId routine.
        """
        self.mechanism = mechanism
        self.config = config
        self.logger = SysIdRoutineLog(name)
    
    def generate_all_tests(self) -> tuple[SysIdTest, SysIdTest, SysIdTest, SysIdTest]:
        """
        Generates all tests and returns a tuple holding them for the user to manually execute.
        """
        quasistatic_forward = SysIdTest(
            direction=Direction.kForward, 
            config=self.config, 
            mechanism=self.mechanism, 
            test_type=TestType.QUASISTATIC,
            logger=self.logger
        )
        quasistatic_reverse = SysIdTest(
            direction=Direction.kReverse, 
            config=self.config, 
            mechanism=self.mechanism, 
            test_type=TestType.QUASISTATIC, 
            logger=self.logger
        )
        dynamic_forward = SysIdTest(
            direction=Direction.kForward, 
            config=self.config, 
            mechanism=self.mechanism, 
            test_type=TestType.DYNAMIC, 
            logger=self.logger
        )
        dynamic_reverse = SysIdTest(
            direction=Direction.kReverse, 
            config=self.config, 
            mechanism=self.mechanism, 
            test_type=TestType.DYNAMIC, 
            logger=self.logger
        )
        return (quasistatic_forward, quasistatic_reverse, dynamic_forward, dynamic_reverse)
