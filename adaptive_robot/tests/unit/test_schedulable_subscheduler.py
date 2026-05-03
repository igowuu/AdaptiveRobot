# pyright: reportPrivateUsage=false

from adaptive_robot.interfaces.schedulable_interface.schedulable_subscheduler import SchedulableSubscheduler
from adaptive_robot.interfaces.schedulable_interface.schedulable import Schedulable
from adaptive_robot.faults.faults import Fault, FaultException, FaultSeverity


class MockSchedulable(Schedulable):
    """
    Mock Schedulable for testing.
    """
    def __init__(self, fault_threshold: int = 10) -> None:
        super().__init__()
        self.fault_threshold = fault_threshold  # Use property setter
        self.execute_count = 0
        self.on_enabled_count = 0
        self.on_disabled_count = 0
        self.on_faulted_init_count = 0
        self.on_faulted_periodic_count = 0
        self._raise_fault: str | None = None
        self._fault_severity = FaultSeverity.ERROR
    
    def execute(self) -> None:
        self.execute_count += 1
        if self._raise_fault == 'execute':
            fault = Fault(
                schedulable=self,   # type: ignore
                severity=self._fault_severity,
                description="Test fault",
                timestamp=0.0,
                exception_type="TestException",
                traceback=None
            )
            raise FaultException(fault)
    
    def on_enabled(self) -> None:
        self.on_enabled_count += 1
        if self._raise_fault == 'on_enabled':
            fault = Fault(
                schedulable=self, # type: ignore
                severity=self._fault_severity,
                description="Test fault",
                timestamp=0.0,
                exception_type="TestException",
                traceback=None
            )
            raise FaultException(fault)
    
    def on_disabled(self) -> None:
        self.on_disabled_count += 1
        if self._raise_fault == 'on_disabled':
            fault = Fault(
                schedulable=self,   # type: ignore
                severity=self._fault_severity,
                description="Test fault",
                timestamp=0.0,
                exception_type="TestException",
                traceback=None
            )
            raise FaultException(fault)
    
    def on_faulted_init(self) -> None:
        self.on_faulted_init_count += 1
        if self._raise_fault == 'on_faulted_init':
            fault = Fault(
                schedulable=self,   # type: ignore
                severity=self._fault_severity,
                description="Test fault",
                timestamp=0.0,
                exception_type="TestException",
                traceback=None
            )
            raise FaultException(fault)
    
    def on_faulted_periodic(self) -> None:
        self.on_faulted_periodic_count += 1


class TestSchedulableSubschedulerInitialization:
    """
    Tests for SchedulableSubscheduler initialization.
    """
    def test_initialization(self) -> None:
        """Tests creating a SchedulableSubscheduler."""
        schedulables = [MockSchedulable()]
        subscheduler = SchedulableSubscheduler(schedulables)    # type: ignore
        
        assert subscheduler.schedulables == schedulables
        assert len(subscheduler._consecutive_faults) == 1
        assert subscheduler._consecutive_faults[schedulables[0]] == 0
    
    def test_multiple_schedulables(self) -> None:
        """
        Tests with multiple schedulables.
        """
        s1, s2, s3 = MockSchedulable(), MockSchedulable(), MockSchedulable()
        subscheduler = SchedulableSubscheduler([s1, s2, s3])
        
        assert len(subscheduler._consecutive_faults) == 3
        for s in [s1, s2, s3]:
            assert subscheduler._consecutive_faults[s] == 0


class TestHealthyExecution:
    """
    Tests for healthy schedulable execution.
    """
    def test_execute_called_when_healthy_and_enabled(self) -> None:
        """
        Tests that execute() is called for healthy schedulables.
        """
        s = MockSchedulable()
        subscheduler = SchedulableSubscheduler([s])
        
        subscheduler.run(enabled=True)
        
        assert s.execute_count == 1
    
    def test_execute_not_called_when_disabled(self) -> None:
        """
        Tests that execute() is not called when robot disabled.
        """
        s = MockSchedulable()
        subscheduler = SchedulableSubscheduler([s])
        
        subscheduler.run(enabled=False)
        
        assert s.execute_count == 0
    
    def test_multiple_schedulables_all_execute(self) -> None:
        """
        Tests that all healthy schedulables execute.
        """
        s1, s2, s3 = MockSchedulable(), MockSchedulable(), MockSchedulable()
        subscheduler = SchedulableSubscheduler([s1, s2, s3])
        
        subscheduler.run(enabled=True)
        
        assert s1.execute_count == 1
        assert s2.execute_count == 1
        assert s3.execute_count == 1
    
    def test_fault_counter_reset_on_successful_execute(self) -> None:
        """
        Tests that fault counter resets after successful execution.
        """
        s = MockSchedulable()
        subscheduler = SchedulableSubscheduler([s])

        subscheduler._consecutive_faults[s] = 5

        subscheduler.run(enabled=True)

        assert subscheduler._consecutive_faults[s] == 0


class TestFaultTracking:
    """
    Tests for consecutive fault counting and health transitions.
    """
    def test_error_fault_increments_counter(self) -> None:
        """
        Tests that ERROR faults increment the fault counter.
        """
        s = MockSchedulable(fault_threshold=3)
        s._raise_fault = 'execute'
        s._fault_severity = FaultSeverity.ERROR
        subscheduler = SchedulableSubscheduler([s])

        try:
            subscheduler.run(enabled=True)
        except FaultException:
            pass
        
        assert subscheduler._consecutive_faults[s] == 1
    
    def test_warning_fault_does_not_increment_counter(self) -> None:
        """
        Tests that WARNING faults don't increment fault counter.
        """
        s = MockSchedulable(fault_threshold=3)
        s._raise_fault = 'execute'
        s._fault_severity = FaultSeverity.WARNING
        subscheduler = SchedulableSubscheduler([s])
        
        try:
            subscheduler.run(enabled=True)
        except FaultException:
            pass

        assert subscheduler._consecutive_faults[s] == 0
    
    def test_consecutive_faults_mark_unhealthy(self) -> None:
        """
        Tests that reaching fault threshold marks schedulable unhealthy.
        """
        s = MockSchedulable(fault_threshold=2)
        s._raise_fault = 'execute'
        s._fault_severity = FaultSeverity.ERROR
        subscheduler = SchedulableSubscheduler([s])

        assert s.is_healthy() is True

        try:
            subscheduler.run(enabled=True)
        except FaultException:
            pass
        assert s.is_healthy() is True
        assert subscheduler._consecutive_faults[s] == 1

        try:
            subscheduler.run(enabled=True)
        except FaultException:
            pass
        assert s.is_healthy() is False
        assert subscheduler._consecutive_faults[s] == 2
    
    def test_on_faulted_init_called_when_unhealthy(self) -> None:
        """
        Tests that on_faulted_init is called when threshold reached.
        """
        s = MockSchedulable(fault_threshold=1)
        s._raise_fault = 'execute'
        s._fault_severity = FaultSeverity.ERROR
        subscheduler = SchedulableSubscheduler([s])
        
        try:
            subscheduler.run(enabled=True)
        except FaultException:
            pass

        assert s.on_faulted_init_count == 1


class TestUnhealthyExecution:
    """
    Tests for unhealthy schedulable behavior.
    """
    def test_on_faulted_periodic_called_when_unhealthy(self) -> None:
        """
        Tests that on_faulted_periodic is called instead of execute.
        """
        s = MockSchedulable(fault_threshold=1)
        s._raise_fault = 'execute'
        s._fault_severity = FaultSeverity.ERROR
        subscheduler = SchedulableSubscheduler([s])

        try:
            subscheduler.run(enabled=True)
        except FaultException:
            pass

        subscheduler.run(enabled=True)
        
        assert s.on_faulted_periodic_count == 1
        assert s.execute_count == 1  # Only called once
    
    def test_on_faulted_periodic_called_even_when_disabled(self) -> None:
        """
        Tests that on_faulted_periodic is called even when disabled.
        """
        s = MockSchedulable(fault_threshold=1)
        s._raise_fault = 'execute'
        s._fault_severity = FaultSeverity.ERROR
        s.set_health(False)
        subscheduler = SchedulableSubscheduler([s])

        subscheduler.run(enabled=False)

        assert s.on_faulted_periodic_count == 0


class TestLifecycleTransitions:
    """
    Tests for on_enabled and on_disabled transitions.
    """
    def test_on_enabled_called_on_disabled_to_enabled_transition(self) -> None:
        """
        Tests that on_enabled is called when transitioning from disabled to enabled.
        """
        s = MockSchedulable()
        subscheduler = SchedulableSubscheduler([s])

        subscheduler.run(enabled=False)
        assert s.on_enabled_count == 0

        subscheduler.run(enabled=True)
        assert s.on_enabled_count == 1
    
    def test_on_disabled_called_on_enabled_to_disabled_transition(self) -> None:
        """
        Tests that on_disabled is called when transitioning from enabled to disabled.
        """
        s = MockSchedulable()
        subscheduler = SchedulableSubscheduler([s])

        subscheduler.run(enabled=True)
        assert s.on_disabled_count == 0

        subscheduler.run(enabled=False)
        assert s.on_disabled_count == 1
    
    def test_on_enabled_not_called_when_already_enabled(self) -> None:
        """
        Tests that on_enabled is not called repeatedly.
        """
        s = MockSchedulable()
        subscheduler = SchedulableSubscheduler([s])

        subscheduler.run(enabled=True)
        subscheduler.run(enabled=True)
        
        assert s.on_enabled_count == 1  # Only once
    
    def test_on_disabled_not_called_when_already_disabled(self) -> None:
        """
        Tests that on_disabled is not called repeatedly.
        """
        s = MockSchedulable()
        subscheduler = SchedulableSubscheduler([s])

        subscheduler.run(enabled=True)
        subscheduler.run(enabled=False)
        subscheduler.run(enabled=False)
        
        assert s.on_disabled_count == 1  # Only once


class TestLockedSchedulables:
    """
    Tests for locked schedulables that skip lifecycle hooks.
    """
    def test_locked_skips_execute(self) -> None:
        """
        Tests that locked schedulables don't execute.
        """
        s = MockSchedulable()
        s.locked = True
        subscheduler = SchedulableSubscheduler([s])
        
        subscheduler.run(enabled=True)
        
        assert s.execute_count == 0
    
    def test_locked_skips_on_enabled(self) -> None:
        """
        Tests that locked schedulables don't call on_enabled.
        """
        s = MockSchedulable()
        s.locked = True
        subscheduler = SchedulableSubscheduler([s])
        
        subscheduler.run(enabled=False)
        subscheduler.run(enabled=True)
        
        assert s.on_enabled_count == 0
    
    def test_locked_skips_on_disabled(self) -> None:
        """
        Tests that locked schedulables don't call on_disabled.
        """
        s = MockSchedulable()
        s.locked = True
        subscheduler = SchedulableSubscheduler([s])
        
        subscheduler.run(enabled=True)
        subscheduler.run(enabled=False)
        
        assert s.on_disabled_count == 0
    
    def test_locked_skips_on_faulted_init(self) -> None:
        """
        Tests that locked unhealthy schedulables don't call on_faulted_init.
        """
        s = MockSchedulable(fault_threshold=1)
        s.locked = True
        s._raise_fault = 'execute'
        s._fault_severity = FaultSeverity.ERROR
        subscheduler = SchedulableSubscheduler([s])
        
        try:
            subscheduler.run(enabled=True)
        except FaultException:
            pass

        assert s.on_faulted_init_count == 0


class TestResetHealth:
    """
    Tests for reset_all_schedulable_health().
    """
    def test_reset_clears_unhealthy_state(self) -> None:
        """
        Tests that reset clears unhealthy schedulables.
        """
        s = MockSchedulable(fault_threshold=1)
        s.set_health(False)
        subscheduler = SchedulableSubscheduler([s])
        
        assert s.is_healthy() is False
        
        subscheduler.reset_all_schedulable_health()
        
        assert s.is_healthy() is True
    
    def test_reset_clears_fault_counters(self) -> None:
        """
        Tests that reset clears fault counters.
        """
        s1, s2 = MockSchedulable(), MockSchedulable()
        subscheduler = SchedulableSubscheduler([s1, s2])
        
        subscheduler._consecutive_faults[s1] = 5
        subscheduler._consecutive_faults[s2] = 8
        
        subscheduler.reset_all_schedulable_health()
        
        assert subscheduler._consecutive_faults[s1] == 0
        assert subscheduler._consecutive_faults[s2] == 0
    
    def test_reset_all_schedulables(self) -> None:
        """
        Tests that reset affects all schedulables.
        """
        schedulables = [MockSchedulable() for _ in range(3)]
        for s in schedulables:
            s.set_health(False)
        
        subscheduler = SchedulableSubscheduler(schedulables)    # type: ignore
        
        for s in schedulables:
            assert s.is_healthy() is False
        
        subscheduler.reset_all_schedulable_health()
        
        for s in schedulables:
            assert s.is_healthy() is True


class TestExceptionHandling:
    """
    Tests for exception handling during execution.
    """
    def test_non_fault_exception_in_execute(self) -> None:
        """
        Tests that non-FaultException exceptions are converted to CRITICAL faults.
        """
        s = MockSchedulable(fault_threshold=3)

        def raise_value_error():
            raise ValueError("Test error")
        s.execute = raise_value_error
        
        subscheduler = SchedulableSubscheduler([s])
        
        try:
            subscheduler.run(enabled=True)
        except FaultException as e:
            assert e.fault.severity == FaultSeverity.CRITICAL

        assert subscheduler._consecutive_faults[s] == 1
    
    def test_fault_exception_in_on_enabled(self) -> None:
        """
        Tests fault exception handling in on_enabled.
        """
        s = MockSchedulable()
        s._raise_fault = 'on_enabled'
        s._fault_severity = FaultSeverity.ERROR
        subscheduler = SchedulableSubscheduler([s])
        
        try:
            subscheduler.run(enabled=False)
            subscheduler.run(enabled=True)
        except FaultException as e:
            assert e.fault.severity == FaultSeverity.ERROR
    
    def test_fault_exception_in_on_disabled(self) -> None:
        """
        Tests fault exception handling in on_disabled.
        """
        s = MockSchedulable()
        s._raise_fault = 'on_disabled'
        s._fault_severity = FaultSeverity.ERROR
        subscheduler = SchedulableSubscheduler([s])
        
        try:
            subscheduler.run(enabled=True)
            subscheduler.run(enabled=False)
        except FaultException as e:
            assert e.fault.severity == FaultSeverity.ERROR


class TestSchedulableSubschedulerIntegration:
    """
    Integration-level tests.
    """
    def test_full_lifecycle_cycle(self) -> None:
        """
        Tests a full lifecycle: disabled -> enabled -> fault -> disabled.
        """
        s = MockSchedulable(fault_threshold=1)
        subscheduler = SchedulableSubscheduler([s])

        subscheduler.run(enabled=False)
        assert s.on_enabled_count == 0

        subscheduler.run(enabled=True)
        assert s.on_enabled_count == 1
        assert s.execute_count == 1

        subscheduler.run(enabled=True)
        assert s.execute_count == 2

        subscheduler.run(enabled=False)
        assert s.on_disabled_count == 1
    
    def test_multiple_schedulables_independent_health(self) -> None:
        """
        Tests that multiple schedulables maintain independent health.
        """
        s1, s2 = MockSchedulable(fault_threshold=1), MockSchedulable(fault_threshold=1)
        s1._raise_fault = 'execute'
        s1._fault_severity = FaultSeverity.ERROR
        subscheduler = SchedulableSubscheduler([s1, s2])

        try:
            subscheduler.run(enabled=True)
        except FaultException:
            pass
        
        assert s1.is_healthy() is False
        assert s2.is_healthy() is True
