from wpimath.units import seconds
from wpimath.controller import PIDController

from adaptive_robot.tunable.tunable_value import TunableValue

class TunablePIDController(PIDController):
    def __init__(
        self, 
        kp: TunableValue[float], 
        ki: TunableValue[float], 
        kd: TunableValue[float], 
        period: seconds = 0.02
    ) -> None:
        super().__init__(kp.value, ki.value, kd.value, period)

        self.kp = kp
        self.ki = ki
        self.kd = kd

    def __str__(self) -> str:
        return f"Kp: {self.kp.value}, Ki: {self.ki.value}, Kd: {self.kd.value}"
    
    def update_from_tunables(self) -> None:
        """
        Checks if any tunable value has changed and updates the PID constants if changed.
        """
        changed = False
        if self.kp.update():
            changed = True
        if self.ki.update():
            changed = True
        if self.kd.update():
            changed = True
        
        if changed:
            self.setPID(self.kp.value, self.ki.value, self.kd.value)
