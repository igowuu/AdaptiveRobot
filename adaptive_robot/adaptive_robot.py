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
    and quality-of-life improvements. Allows for helpful methods like tunable() and
    automatic execution of execute() and publish_telementry() if included within the components code
    to reduce boilerplate.

    Allows (and heavily encourages) a request-based flow, which is iterated through other files
    in this directory.
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
            component.publish_telemetry()
            component.execute()

        self._update_components_post_loop()

        self.onRobotPeriodic()

    def onRobotInit(self) -> None:
        pass

    def onRobotPeriodic(self) -> None:
        pass