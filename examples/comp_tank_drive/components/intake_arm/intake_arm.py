from wpimath.units import radians
from wpimath.controller import ArmFeedforward

from components.intake_arm.intake_arm_constants import IntakeArmFF, IntakeArmPID
from components.intake_arm.intake_arm_io.io_base import IntakeArmIOBase
from components.intake_arm.intake_arm_constants import IntakeArmConstants

from adaptive_robot import AdaptiveComponent, RequestArbitrator, BasicPriority
from adaptive_robot.utils.math_utils import clamp


class IntakeArm(AdaptiveComponent):
    """
    Represents the intake arm mechanism of a robot (single joint).
    Uses PID control to reach requested angles with gravity compensation.
    """
    def __init__(self, io: IntakeArmIOBase) -> None:
        self.io = io

        self.arm_position_ff = ArmFeedforward(
            kS=IntakeArmFF.KS,
            kG=IntakeArmFF.KG,
            kV=IntakeArmFF.KV,
            kA=IntakeArmFF.KA
        )
        self.arm_position_pid = self.tunablePID(
            kp=IntakeArmPID.KP, 
            ki=IntakeArmPID.KI, 
            kd=IntakeArmPID.KD,
            directory="Tunables/IntakeArmPID"
        )

        self.angle_controller = RequestArbitrator()

    def request_angle(
        self,
        angle: radians,
        priority: BasicPriority, 
        source: str = "unknown"
    ) -> None:
        """
        Requests a velocity to the intake arms angle controller.
        """
        self.angle_controller.request(angle, priority.value, source)

    def _safe_defaults(self) -> None:
        """
        Directly commands safe default values to the intake arm.
        """
        self.io.set_voltage(0.0)

    def on_enabled(self) -> None:
        """
        Method called once when the robot enters a enabled state.
        """
        self._safe_defaults()

    def on_disabled(self) -> None:
        """
        Method called once when the robot enters a disabled state.
        """
        self._safe_defaults()
    
    def on_faulted_init(self) -> None:
        """
        Runs once when the component first becomes unhealthy.
        """
        self._safe_defaults()
    
    def on_faulted_periodic(self) -> None:
        """
        Runs each iteration when the component is unhealthy.
        """
        self._safe_defaults()

    def publish_telemetry(self) -> None:
        """
        Runs each iteration before component execution, regardless of component health.
        """
        self.publish_value("IntakeArm/sensorData/position", self.io.get_position())
        self.publish_value("IntakeArm/sensorData/velocity", self.io.get_velocity())
        self.publish_value("IntakeArm/sensorData/appliedVoltage", self.io.get_voltage())

        resolved = self.angle_controller.resolve()
        self.publish_value("IntakeArm/resolvedAngle/angle", resolved.value)
        self.publish_value("IntakeArm/resolvedAngle/source", resolved.source)
        self.publish_value("IntakeArm/resolvedAngle/priority", resolved.priority)

    def _move_to_angle(self, angle: radians) -> None:
        """
        Moves the intake arm to an angle via closed loop control.
        """
        current_angle = self.io.get_position()
        current_velocity = self.io.get_velocity()

        ff_volts = self.arm_position_ff.calculate(current_angle, current_velocity)
        pid_volts = self.arm_position_pid.calculate(current_angle, angle)

        self.io.set_voltage(ff_volts + pid_volts)

    def execute(self) -> None:
        """
        Directly moves the robot each iteration if the component is healthy.
        """
        target_angle = self.angle_controller.resolve().value
        target_angle = clamp(target_angle, IntakeArmConstants.MIN_ANGLE, IntakeArmConstants.MAX_ANGLE)

        self._move_to_angle(target_angle)
        self.io.update()
