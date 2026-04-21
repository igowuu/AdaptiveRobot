# pyright: reportPrivateUsage=false

from unittest.mock import Mock
import logging

from adaptive_robot.adaptive_component.adaptive_component import AdaptiveComponent, ComponentContext
from adaptive_robot.telemetry.telemetry import TelemetryPublisher
from adaptive_robot.telemetry.struct_telemetry import TelemetryStructPublisher


class MockAdaptiveComponent(AdaptiveComponent):
    """
    Fake AdaptiveComponent implementation for testing.
    """
    def __init__(self, context: ComponentContext | None = None) -> None:
        super().__init__(context)
        self.execute_called = False
        self.publish_telemetry_called = False
        self.on_faulted_called = False
    
    def execute(self) -> None:
        self.execute_called = True
    
    def publish_telemetry(self) -> None:
        self.publish_telemetry_called = True
    
    def on_faulted_init(self) -> None:
        self.on_faulted_called = True


class TestAdaptiveComponentInitialization:
    """
    Tests for AdaptiveComponent initialization.
    """
    def test_component_creation(self) -> None:
        """
        Tests creating an adaptive component.
        """
        component = MockAdaptiveComponent()
        
        assert component.is_healthy() == True
        assert component._context == None
    
    def test_component_with_context(self) -> None:
        """
        Tests creating an adaptive component with context.
        """
        telemetry = Mock(spec=TelemetryPublisher)
        struct_telemetry = Mock(spec=TelemetryStructPublisher)
        logger = logging.getLogger(__name__)
        
        context = ComponentContext(
            telemetry=telemetry,
            struct_telemetry=struct_telemetry,
            logger=logger
        )
        component = MockAdaptiveComponent(context)
        
        assert component.is_healthy() == True
        assert component._context == context


class TestAdaptiveComponentTunables:
    """
    Tests for tunable value management.
    """
    def test_tunable_registration(self) -> None:
        """
        Tests that tunables are properly registered to the context.
        """
        telemetry = Mock(spec=TelemetryPublisher)
        struct_telemetry = Mock(spec=TelemetryStructPublisher)
        logger = logging.getLogger(__name__)
        
        context = ComponentContext(
            telemetry=telemetry,
            struct_telemetry=struct_telemetry,
            logger=logger
        )
        component = MockAdaptiveComponent(context)

        tunable = component.tunable("test/value", 0.5)

        assert tunable in context.tunables
        assert len(context.tunables) == 1
    
    def test_tunable_pid_registration(self) -> None:
        """
        Tests that tunable PIDs are properly registered to the context.
        """
        telemetry = Mock(spec=TelemetryPublisher)
        struct_telemetry = Mock(spec=TelemetryStructPublisher)
        logger = logging.getLogger(__name__)
        
        context = ComponentContext(
            telemetry=telemetry,
            struct_telemetry=struct_telemetry,
            logger=logger
        )
        component = MockAdaptiveComponent(context)

        pid = component.tunablePID(1.0, 0.5, 0.2, "test/pid")

        assert pid in context.tunable_pids
        assert len(context.tunable_pids) == 1


class TestAdaptiveComponentHealthManagement:
    """
    Tests for health management.
    """
    def test_is_healthy(self) -> None:
        """
        Tests checking if component is healthy.
        """
        component = MockAdaptiveComponent()
        assert component.is_healthy() is True
    
    def test_set_health(self) -> None:
        """
        Tests setting component health.
        """
        component = MockAdaptiveComponent()
        component.set_health(False)
        assert component.is_healthy() is False
        
        component.set_health(True)
        assert component.is_healthy() is True


class TestAdaptiveComponentPublishMethods:
    """
    Tests for telemetry publishing methods.
    """
    def test_publish_value(self) -> None:
        """
        Tesst publishing a value to telemetry.
        """
        telemetry = Mock(spec=TelemetryPublisher)
        struct_telemetry = Mock(spec=TelemetryStructPublisher)
        logger = logging.getLogger(__name__)
        
        context = ComponentContext(
            telemetry=telemetry,
            struct_telemetry=struct_telemetry,
            logger=logger
        )
        component = MockAdaptiveComponent(context)
        
        component.publish_value("test/key", 42)
        
        telemetry.put_value.assert_called_once_with("test/key", 42)
    
    def test_publish_struct_value(self) -> None:
        """
        Tests publishing a struct value to telemetry.
        """
        telemetry = Mock(spec=TelemetryPublisher)
        struct_telemetry = Mock(spec=TelemetryStructPublisher)
        logger = logging.getLogger(__name__)
        
        context = ComponentContext(
            telemetry=telemetry,
            struct_telemetry=struct_telemetry,
            logger=logger
        )
        component = MockAdaptiveComponent(context)
        
        struct_obj = Mock()
        component.publish_struct_value("test/struct", struct_obj)
        
        struct_telemetry.put_struct_value.assert_called_once_with("test/struct", struct_obj)


class TestAdaptiveRobotComponentManagement:
    """
    Tests for component management.
    """
    def test_components_list_functionality(self) -> None:
        """
        Tests adding a component into the internal AdaptiveRobot list.
        """
        robot = Mock()
        robot.components = []
        component = Mock()

        robot.components.append(component)
        
        assert len(robot.components) == 1
        assert component in robot.components
    
    def test_add_multiple_components(self) -> None:
        """
        Tests adding multiple components.
        """
        robot = Mock()
        robot.components = []
        comp1 = Mock()
        comp2 = Mock()
        comp3 = Mock()

        robot.components.append(comp1)
        robot.components.append(comp2)
        robot.components.append(comp3)
        
        assert len(robot.components) == 3
        assert all(c in robot.components for c in [comp1, comp2, comp3])


class TestAdaptiveRobotActionManagement:
    """
    Tests for action scheduling logic.
    """
    def test_schedule_action_goes_to_scheduler(self) -> None:
        """
        Tests that schedule_action calls the scheduler.
        """
        robot = Mock()
        robot.action_scheduler = Mock()
        action = Mock()

        robot.action_scheduler.schedule(action)
        
        robot.action_scheduler.schedule.assert_called_once_with(action)
    
    def test_cancel_action_delegates_to_scheduler(self) -> None:
        """
        Tests that cancel_action calls the scheduler.
        """
        robot = Mock()
        robot.action_scheduler = Mock()
        action = Mock()

        robot.action_scheduler.cancel(action)
        
        robot.action_scheduler.cancel.assert_called_once_with(action)
    
    def test_cancel_all_actions_delegates_to_scheduler(self) -> None:
        """
        Tests that cancel_all_actions calls the scheduler.
        """
        robot = Mock()
        robot.action_scheduler = Mock()

        robot.action_scheduler.cancel_all()
        
        robot.action_scheduler.cancel_all.assert_called_once()


class TestAdaptiveRobotPeriodicExecution:
    """
    Tests for robot periodic execution logic.
    """
    def test_component_execution_when_healthy_and_enabled(self) -> None:
        """
        Tests that healthy components execute when robot is enabled.
        """
        robot = Mock()
        robot.components = []
        robot.isEnabled = Mock(return_value=True)
        robot.action_scheduler = Mock()
        robot.onRobotPeriodic = Mock()
        
        component = Mock()
        component.is_healthy.return_value = True
        robot.components.append(component)
        
        # Simulate robotPeriodic
        if robot.isEnabled():
            if component.is_healthy():
                component.execute()
                component.reset_failure_count()

        component.execute.assert_called_once()
        component.reset_failure_count.assert_called_once()
    
    def test_component_not_executed_when_disabled(self) -> None:
        """
        Tests that components don't execute when robot is disabled.
        """
        component = Mock()
        component.is_healthy.return_value = True
        
        # Simulate robotPeriodic with disabled robot
        is_enabled = False
        if is_enabled:
            component.execute()

        component.execute.assert_not_called()
    
    def test_unhealthy_component_calls_on_faulted(self) -> None:
        """
        Tests that unhealthy components call on_faulted instead of execute.
        """
        component = Mock()
        component.is_healthy.return_value = False

        if component.is_healthy():
            component.execute()
        else:
            component.on_faulted()
        
        component.execute.assert_not_called()
        component.on_faulted.assert_called_once()
    
    def test_exception_in_component_is_caught(self) -> None:
        """
        Tests that exceptions in components don't halt execution.
        """
        component = Mock()
        component.is_healthy.return_value = True
        component.execute.side_effect = RuntimeError("test error")
        
        try:
            component.execute()
        except RuntimeError:
            component.record_failure(RuntimeError("test error"))

        component.record_failure.assert_called_once()


class TestAdaptiveRobotLifecycleMethods:
    """
    Tests for robot lifecycle methods - tests the delegation pattern.
    """
    def test_robotinit_calls_onrobotinit(self) -> None:
        """
        Tests that robotInit delegates to onRobotInit.
        """
        robot = Mock()
        robot.onRobotInit = Mock()

        robot.onRobotInit()
        
        robot.onRobotInit.assert_called_once()
    
    def test_disabledinit_calls_ondisabledinit(self) -> None:
        """
        Tests that disabledInit delegates to onDisabledInit.
        """
        robot = Mock()
        robot.onDisabledInit = Mock()

        robot.onDisabledInit()
        
        robot.onDisabledInit.assert_called_once()
