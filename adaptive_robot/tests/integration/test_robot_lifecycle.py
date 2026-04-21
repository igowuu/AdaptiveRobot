# pyright: reportPrivateUsage=false

import tempfile
from unittest.mock import MagicMock

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.adaptive_component.adaptive_component import AdaptiveComponent


class LifecycleTrackingComponent(AdaptiveComponent):
    def __init__(self, name: str = "test_component") -> None:
        super().__init__()
        self.name = name
        self.lifecycle_events: list[str] = []
    
    def execute(self) -> None:
        self.lifecycle_events.append("execute")
    
    def publish_telemetry(self) -> None:
        self.lifecycle_events.append("publish_telemetry")
    
    def on_enabled(self) -> None:
        self.lifecycle_events.append("on_enabled")
    
    def on_disabled(self) -> None:
        self.lifecycle_events.append("on_disabled")
    
    def on_faulted_init(self) -> None:
        self.lifecycle_events.append("on_faulted_init")
    
    def on_faulted_periodic(self) -> None:
        self.lifecycle_events.append("on_faulted_periodic")


class RobotWithMocking(AdaptiveRobot):
    def __init__(self, period: float = 0.02, fault_logging_folder: str = "faults/") -> None:
        self.test_component = LifecycleTrackingComponent("main")
        super().__init__(period, fault_logging_folder)
        self.on_robot_init_called = False
        self.on_robot_periodic_called = False
        self.on_disabled_init_called = False
        self.on_teleop_init_called = False
        self.on_autonomous_init_called = False
    
    def onRobotInit(self) -> None:
        self.on_robot_init_called = True
    
    def onRobotPeriodic(self) -> None:
        self.on_robot_periodic_called = True
    
    def onDisabledInit(self) -> None:
        self.on_disabled_init_called = True
    
    def onTeleopInit(self) -> None:
        self.on_teleop_init_called = True
    
    def onAutonomousInit(self) -> None:
        self.on_autonomous_init_called = True


class TestRobotInitialization:
    def test_robot_init_calls_on_robot_init_hook(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            robot = RobotWithMocking(fault_logging_folder=tmpdir)
            robot.robotInit()
            assert robot.on_robot_init_called is True
            assert robot._scheduler_initialized is True
    
    def test_robot_auto_registers_components(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            robot = RobotWithMocking(fault_logging_folder=tmpdir)
            assert robot.test_component not in robot._components
            robot.robotInit()
            assert robot.test_component in robot._components
            assert len(robot._components) == 1
    
    def test_component_scheduler_initialized(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            robot = RobotWithMocking(fault_logging_folder=tmpdir)
            assert robot._component_scheduler is None
            robot.robotInit()
            assert robot._component_scheduler is not None
            assert len(robot._component_scheduler.components) == 1


class TestComponentTransitions:
    def test_on_enabled_called_on_enable_transition(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            robot = RobotWithMocking(fault_logging_folder=tmpdir)
            robot.robotInit()
            robot.isEnabled = MagicMock(return_value=True)
            robot.robotPeriodic()
            assert "on_enabled" in robot.test_component.lifecycle_events
    
    def test_component_executes_when_enabled(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            robot = RobotWithMocking(fault_logging_folder=tmpdir)
            robot.robotInit()
            robot.isEnabled = MagicMock(return_value=True)
            robot.robotPeriodic()
            assert "execute" in robot.test_component.lifecycle_events
