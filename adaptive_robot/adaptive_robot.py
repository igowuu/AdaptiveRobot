# AdaptiveRobot - FRC architecture
# Copyright (c) 2026 Jacob Taylor (igowu) <https://github.com/igowuu>
# Code from: <https://github.com/igowuu/AdaptiveRobot.git>

# Licensed under the MIT License.
# See https://opensource.org/licenses/MIT for details.

from typing import final

from wpilib import TimedRobot

from adaptive_robot.telemetry import TelemetryPublisher
from adaptive_robot.adaptive_component import AdaptiveComponent


class AdaptiveRobot(TimedRobot):
    """
    Wrapper for TimedRobot that allows for a dynamic structure while enforcing safety measures 
    and quality-of-life improvements.
    """
    def __init__(self) -> None:
        super().__init__()
        self.telemetry_publisher = TelemetryPublisher()
        self.components: list["AdaptiveComponent"] = []

    @final
    def add_component(self, component: "AdaptiveComponent") -> None:
        self.components.append(component)

    @final
    def robotInit(self) -> None:
        self.onRobotInit()

    def _update_components_post_loop(self) -> None:
        for component in self.components:
            component.update_tunable_constants()
            component.update_tunable_pids()

    @final
    def robotPeriodic(self) -> None:
        for component in self.components:
            try:
                component.publish_telemetry()
                component.execute()
            except Exception:
                raise

        self._update_components_post_loop()

        self.onRobotPeriodic()

    def onRobotInit(self) -> None:
        """
        Robot-wide initialization code should go here.

        Users should override this method for default Robot-wide initialization which will be called 
        when the robot is first powered on. 
        It will be called exactly one time.
        """
        pass

    def onRobotPeriodic(self) -> None:
        """
        Periodic code for all modes should go here.

        This function is called each time a new packet is received from the driver station.
        """
        pass
