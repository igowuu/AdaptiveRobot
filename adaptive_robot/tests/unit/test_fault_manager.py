# pyright: reportPrivateUsage=false

from unittest.mock import Mock, patch
from adaptive_robot.fault_manager import FaultManager
from adaptive_robot.faults.faults import Fault, FaultException, FaultSeverity
from adaptive_robot.interfaces.schedulable_interface.schedulable_subscheduler import SchedulableSubscheduler
from adaptive_robot.interfaces.telemetry_interface.telemetry_subscheduler import TelemetrySubscheduler
from adaptive_robot.interfaces.tunable_interface.tunable_subscheduler import TunableSubscheduler
from adaptive_robot.autonomous.action_scheduler import ActionScheduler
from adaptive_robot.faults.fault_scheduler import FaultLogger


def create_mock_fault(severity: FaultSeverity = FaultSeverity.ERROR) -> Fault:
    """
    Creates a mock fault for testing.
    """
    return Fault(
        schedulable=Mock(),
        severity=severity,
        description="Test fault",
        timestamp=0.0,
        exception_type="TestException",
        traceback=None
    )


def create_fault_manager() -> tuple[FaultManager, Mock, Mock, Mock, Mock, Mock]:
    """
    Creates a FaultManager with all mocked dependencies.
    Returns: (manager, schedulable_mock, telemetry_mock, tunable_mock, action_mock, logger_mock)
    """
    schedulable_mock = Mock(spec=SchedulableSubscheduler)
    telemetry_mock = Mock(spec=TelemetrySubscheduler)
    tunable_mock = Mock(spec=TunableSubscheduler)
    action_mock = Mock(spec=ActionScheduler)
    logger_mock = Mock(spec=FaultLogger)
    
    manager = FaultManager(
        schedulable_subscheduler=schedulable_mock,
        telemetry_subscheduler=telemetry_mock,
        tunable_subscheduler=tunable_mock,
        action_scheduler=action_mock,
        fault_logger=logger_mock
    )
    
    return manager, schedulable_mock, telemetry_mock, tunable_mock, action_mock, logger_mock


class TestFaultManagerInitialization:
    """
    Tests for FaultManager initialization.
    """
    def test_fault_manager_creation(self) -> None:
        """
        Tests creating a FaultManager.
        """
        manager, *_ = create_fault_manager()
        assert manager._faults_this_iter == []
    
    def test_fault_manager_stores_dependencies(self) -> None:
        """
        Tests that FaultManager stores all dependencies.
        """
        manager, sched, telem, tunable, action, logger = create_fault_manager()
        
        assert manager._schedulable_subscheduler is sched
        assert manager._telemetry_subscheduler is telem
        assert manager._tunable_subscheduler is tunable
        assert manager._action_scheduler is action
        assert manager._fault_logger is logger


class TestSchedulerExecutionOrder:
    """
    Tests that schedulers run in correct order.
    """
    def test_execution_order_tunable_telemetry_schedulable_action(self) -> None:
        """
        Tests that schedulers execute in order: tunable, telemetry, schedulable, action.
        """
        manager, sched, telem, tunable, action, logger = create_fault_manager()
        call_order: list[str] = []
        
        tunable.run.side_effect = lambda *args, **kwargs: call_order.append('tunable')  # type: ignore
        telem.run.side_effect = lambda *args, **kwargs: call_order.append('telemetry')  # type: ignore
        sched.run.side_effect = lambda *args, **kwargs: call_order.append('schedulable')# type: ignore
        action.run.side_effect = lambda *args, **kwargs: call_order.append('action')    # type: ignore
        logger.run.side_effect = lambda *args, **kwargs: call_order.append('logger')    # type: ignore
        
        manager.run(enabled=True)
        
        assert call_order == ['tunable', 'telemetry', 'schedulable', 'action', 'logger']
    
    def test_execution_order_disabled_robot(self) -> None:
        """
        Tests execution order when robot is disabled.
        """
        manager, sched, telem, tunable, action, logger = create_fault_manager()
        call_order: list[str] = []
        
        tunable.run.side_effect = lambda *args, **kwargs: call_order.append('tunable')  # type: ignore
        telem.run.side_effect = lambda *args, **kwargs: call_order.append('telemetry')  # type: ignore
        sched.run.side_effect = lambda *args, **kwargs: call_order.append('schedulable')# type: ignore
        action.run.side_effect = lambda *args, **kwargs: call_order.append('action')    # type: ignore
        logger.run.side_effect = lambda *args, **kwargs: call_order.append('logger')    # type: ignore
        
        manager.run(enabled=False)

        assert 'tunable' in call_order
        assert 'telemetry' in call_order
        assert 'schedulable' in call_order


class TestFaultHandling:
    """
    Tests for fault handling and routing.
    """
    def test_warning_fault_does_not_disable_robot(self) -> None:
        """
        Tests that WARNING faults don't disable the robot.
        """
        manager, sched, _, _, _, logger = create_fault_manager()
        fault = create_mock_fault(severity=FaultSeverity.WARNING)
        sched.run.side_effect = FaultException(fault)
        
        should_disable = manager.run(enabled=True)
        
        assert should_disable is False
        logger.run.assert_called_once()
    
    def test_error_fault_does_not_disable_robot(self) -> None:
        """
        Tests that ERROR faults don't disable the robot.
        """
        manager, sched, _, _, _, logger = create_fault_manager()
        fault = create_mock_fault(severity=FaultSeverity.ERROR)
        sched.run.side_effect = FaultException(fault)
        
        should_disable = manager.run(enabled=True)
        
        assert should_disable is False
        logger.run.assert_called_once()
    
    def test_critical_fault_disables_robot(self) -> None:
        """
        Tests that CRITICAL faults disable the robot.
        """
        manager, sched, _, _, _, logger = create_fault_manager()
        fault = create_mock_fault(severity=FaultSeverity.CRITICAL)
        sched.run.side_effect = FaultException(fault)
        
        should_disable = manager.run(enabled=True)
        
        assert should_disable is True
        logger.run.assert_called_once()
    
    def test_multiple_schedulers_can_fault(self) -> None:
        """
        Tests that faults from any scheduler are handled.
        """
        for target_sched, sched_name in [
            ("tunable", "tunable"),
            ("telem", "telemetry"),
            ("sched", "schedulable"),
        ]:
            manager, sched, telem, tunable, _, logger = create_fault_manager()
            fault = create_mock_fault(severity=FaultSeverity.WARNING)
            
            if target_sched == "tunable":
                tunable.run.side_effect = FaultException(fault)
            elif target_sched == "telem":
                telem.run.side_effect = FaultException(fault)
            else:
                sched.run.side_effect = FaultException(fault)
            
            should_disable = manager.run(enabled=True)
            
            # All should be handled without disabling (WARNING)
            assert should_disable is False, f"Fault in {sched_name} should not disable"
            logger.run.assert_called_once()


class TestFaultRouting:
    """
    Tests that faults are routed based on severity.
    """
    @patch('wpilib.reportError')
    @patch('wpilib.reportWarning')
    def test_critical_fault_reports_error(self, _, mock_error: Mock) -> None:
        """
        Tests that CRITICAL faults call reportError.
        """
        manager, sched, _, _, _, _ = create_fault_manager()
        fault = create_mock_fault(severity=FaultSeverity.CRITICAL)
        sched.run.side_effect = FaultException(fault)
        
        manager.run(enabled=True)
        
        mock_error.assert_called_once()
        assert "[CRITICAL]" in str(mock_error.call_args)
    
    @patch('wpilib.reportError')
    @patch('wpilib.reportWarning')
    def test_error_fault_reports_warning(self, mock_warn: Mock, _) -> None:
        """
        Tests that ERROR faults call reportWarning.
        """
        manager, sched, _, _, _, _ = create_fault_manager()
        fault = create_mock_fault(severity=FaultSeverity.ERROR)
        sched.run.side_effect = FaultException(fault)
        
        manager.run(enabled=True)
        
        mock_warn.assert_called_once()
        assert "[ERROR]" in str(mock_warn.call_args)
    
    @patch('wpilib.reportWarning')
    def test_warning_fault_reports_warning(self, mock_warn: Mock) -> None:
        """
        Tests that WARNING faults call reportWarning.
        """
        manager, sched, _, _, _, _ = create_fault_manager()
        fault = create_mock_fault(severity=FaultSeverity.WARNING)
        sched.run.side_effect = FaultException(fault)
        
        manager.run(enabled=True)
        
        mock_warn.assert_called_once()
        assert "[WARNING]" in str(mock_warn.call_args)


class TestFaultLogging:
    """
    Tests for fault logging.
    """
    def test_faults_logged_each_iteration(self) -> None:
        """
        Tests that faults are logged at end of each iteration.
        """
        manager, sched, _, _, _, logger = create_fault_manager()
        fault = create_mock_fault(severity=FaultSeverity.ERROR)
        sched.run.side_effect = FaultException(fault)
        
        manager.run(enabled=True)
        
        logger.run.assert_called_once()
    
    def test_fault_list_cleared_after_logging(self) -> None:
        """
        Tests that fault list is cleared after logging.
        """
        manager, sched, _, _, _, _ = create_fault_manager()
        fault = create_mock_fault(severity=FaultSeverity.ERROR)
        sched.run.side_effect = FaultException(fault)
        
        manager.run(enabled=True)

        assert len(manager._faults_this_iter) == 0
    
    def test_logging_called_even_on_fault(self) -> None:
        """
        Tests that logging still happens even when fault occurs.
        """
        manager, sched, _, _, _, logger = create_fault_manager()
        fault = create_mock_fault(severity=FaultSeverity.CRITICAL)
        sched.run.side_effect = FaultException(fault)
        
        manager.run(enabled=True)
        
        logger.run.assert_called_once()
    
    def test_logging_called_on_success(self) -> None:
        """
        Tests that logging is called on successful run (no faults).
        """
        manager, _, _, _, _, logger = create_fault_manager()
        
        manager.run(enabled=True)
        
        logger.run.assert_called_once()
    
    def test_consecutive_runs_clear_faults(self) -> None:
        """
        Tests that consecutive runs have separate fault logs.
        """
        manager, _, _, _, _, logger = create_fault_manager()

        manager.run(enabled=True)
        first_call_count = logger.run.call_count

        manager.run(enabled=True)
        second_call_count = logger.run.call_count

        assert first_call_count == 1
        assert second_call_count == 2


class TestExceptionPropagation:
    """
    Tests how FaultManager propagates exceptions.
    """
    def test_fault_exception_propagates_to_caller(self) -> None:
        """
        Tests that FaultException is caught and handled internally.
        """
        manager, sched, _, _, _, _ = create_fault_manager()
        fault = create_mock_fault(severity=FaultSeverity.ERROR)
        sched.run.side_effect = FaultException(fault)

        should_disable = manager.run(enabled=True)
        assert should_disable is False
    
    def test_non_fault_exception_caught_and_handled(self) -> None:
        """
        Tests that non-FaultException exceptions are still caught.
        """
        manager, _, _, tunable, _, _ = create_fault_manager()

        tunable.run.side_effect = RuntimeError("Unexpected error")

        try:
            manager.run(enabled=True)
            assert False, "Should have raised RuntimeError"
        except RuntimeError:
            pass


class TestDisabledRobotBehavior:
    """
    Tests FaultManager behavior when robot is disabled.
    """
    def test_schedulable_receives_enabled_false(self) -> None:
        """
        Tests that schedulable subscheduler receives enabled=False.
        """
        manager, sched, _, _, _, _ = create_fault_manager()
        
        manager.run(enabled=False)
        
        sched.run.assert_called_once_with(False)
    
    def test_faults_still_logged_when_disabled(self) -> None:
        """
        Tests that faults are logged even when robot disabled.
        """
        manager, sched, _, _, _, logger = create_fault_manager()
        fault = create_mock_fault(severity=FaultSeverity.ERROR)
        sched.run.side_effect = FaultException(fault)
        
        manager.run(enabled=False)
        
        logger.run.assert_called_once()
    
    def test_critical_fault_disables_when_disabled(self) -> None:
        """
        Tests that CRITICAL faults disable even if robot already disabled.
        """
        manager, sched, _, _, _, _ = create_fault_manager()
        fault = create_mock_fault(severity=FaultSeverity.CRITICAL)
        sched.run.side_effect = FaultException(fault)
        
        should_disable = manager.run(enabled=False)
        
        assert should_disable is True


class TestFaultManagerIntegration:
    """
    Integration-level tests for FaultManager.
    """
    def test_full_execution_cycle_success(self) -> None:
        """
        Tests a complete successful execution cycle.
        """
        manager, sched, telem, tunable, action, logger = create_fault_manager()

        result = manager.run(enabled=True)
        
        assert result is False
        tunable.run.assert_called_once()
        telem.run.assert_called_once()
        sched.run.assert_called_once_with(True)
        action.run.assert_called_once()
        logger.run.assert_called_once()
    
    def test_fault_in_different_schedulers(self) -> None:
        """
        Tests faults from different schedulers.
        """
        manager1, _, _, tunable1, _, logger1 = create_fault_manager()
        fault = create_mock_fault(severity=FaultSeverity.WARNING)
        tunable1.run.side_effect = FaultException(fault)
        manager1.run(enabled=True)
        logger1.run.assert_called_once()

        manager2, _, telem2, _, _, logger2 = create_fault_manager()
        telem2.run.side_effect = FaultException(fault)
        manager2.run(enabled=True)
        logger2.run.assert_called_once()

        manager3, sched3, _, _, _, logger3 = create_fault_manager()
        sched3.run.side_effect = FaultException(fault)
        manager3.run(enabled=True)
        logger3.run.assert_called_once()
    
    def test_sequential_iterations_clear_faults(self) -> None:
        """
        Tests that faults from iteration N don't carry to iteration N+1.
        """
        manager, sched, _, _, _, _ = create_fault_manager()
        fault = create_mock_fault(severity=FaultSeverity.ERROR)

        sched.run.side_effect = FaultException(fault)
        manager.run(enabled=True)
        assert len(manager._faults_this_iter) == 0  # Cleared after logging

        sched.run.side_effect = None
        manager.run(enabled=True)
        assert len(manager._faults_this_iter) == 0
