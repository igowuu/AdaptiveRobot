from wpimath.units import seconds
from wpimath.controller import PIDController

import wpilib

from adaptive_robot.tunable.tunable_value import TunableValue


class TunablePIDController(PIDController):
    def __init__(
        self, 
        kp: TunableValue[float], 
        ki: TunableValue[float], 
        kd: TunableValue[float], 
        period: seconds = 0.02
    ) -> None:
        if period <= 0:
            raise ValueError(
                f"period must be positive, got {period}. "
                f"This parameter controls the update rate of the PID controller in seconds."
            )
        
        super().__init__(kp.value, ki.value, kd.value, period)

        if kp.value < 0:
            wpilib.reportWarning(
                f"PID constant kp has been set to a value less "
                f"than zero ({kp.value}) during initialization."
            )
        if ki.value < 0:
            wpilib.reportWarning(
                f"PID constant ki has been set to a value less "
                f"than zero ({ki.value}) during initialization."
            )
        if kd.value < 0:
            wpilib.reportWarning(
                f"PID constant kd has been set to a value less "
                f"than zero ({kd.value}) during initialization."
            )

        self._kp = kp
        self._ki = ki
        self._kd = kd

    def __str__(self) -> str:
        return f"Kp: {self._kp.value}, Ki: {self._ki.value}, Kd: {self._kd.value}"
    
    def update_from_tunables(self) -> None:
        """
        Checks if any tunable value has changed and updates the PID constants if changed.
        """
        changed = False
        if self._kp.update():
            changed = True
        if self._ki.update():
            changed = True
        if self._kd.update():
            changed = True
        
        if changed:
            self.setPID(self._kp.value, self._ki.value, self._kd.value)

            if self._kp.value < 0:
                wpilib.reportWarning(
                    f"PID constant kp has been set to a value less "
                    f"than zero ({self._kp.value}) during runtime."
                )
            if self._ki.value < 0:
                wpilib.reportWarning(
                    f"PID constant ki has been set to a value less "
                    f"than zero ({self._ki.value}) during runtime."
                )
            if self._kd.value < 0:
                wpilib.reportWarning(
                    f"PID constant kd has been set to a value less "
                    f"than zero ({self._kd.value}) during runtime."
                )
