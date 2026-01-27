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
    Wrapper for TimedRobot that allows for a dynamic structure while still enforcing
    safety measures and quality-of-life improvements. The best of CommandRobot and MagicRobot.
    Allows for extremely helpful methods like tunable() and will_reset_to() to reduce bugs.
    Components are not equivelent to subsystems, but rather any file directly correlated to the bot.

    Note that components are executed in the order that they are constructed.
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