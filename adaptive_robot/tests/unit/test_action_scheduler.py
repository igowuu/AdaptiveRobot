# pyright: reportPrivateUsage=false


from adaptive_robot.autonomous.action_scheduler import ActionScheduler
from adaptive_robot.autonomous.async_actions import AsyncAction
from adaptive_robot.faults.faults import FaultSeverity, FaultException, Fault


def create_mock_action(send_count: int = 1) -> AsyncAction:
    """
    Creates a mock async action that completes after send_count send() calls.
    """
    for _ in range(send_count - 1):
        yield


def create_infinite_action() -> AsyncAction:
    """
    Creates an async action that never completes.
    """
    while True:
        yield


def create_error_action(error_on_send: int = 1) -> AsyncAction:
    """
    Creates an async action that raises after error_on_send send() calls.
    """
    for _ in range(error_on_send - 1):
        yield
    raise ValueError("Test error in action")


def create_fault_exception_action() -> AsyncAction:
    """
    Creates an async action that raises a FaultException.
    """
    yield
    fault = Fault(
        component=None,
        severity=FaultSeverity.CRITICAL,
        description="Test fault exception",
        timestamp=0.0,
        exception_type="TestException",
        traceback=None
    )
    raise FaultException(fault)


class TestActionSchedulerInitialization:
    """
    Tests for ActionScheduler initialization.
    """
    def test_scheduler_creation(self) -> None:
        """
        Tests creating an action scheduler instance.
        """
        scheduler = ActionScheduler()
        assert scheduler is not None
        assert len(scheduler.get_running_actions()) == 0

    def test_scheduler_internal_state(self) -> None:
        """
        Tests that scheduler correctly initializes internal state.
        """
        scheduler = ActionScheduler()
        assert scheduler._actions == {}


class TestActionSchedulerScheduling:
    """
    Tests for scheduling actions in the scheduler.
    """
    def test_schedule_single_action(self) -> None:
        """
        Tests scheduling a single action.
        """
        scheduler = ActionScheduler()
        action = create_mock_action(send_count=2)

        result = scheduler.schedule(action, "test_action")

        assert result == "test_action"
        assert scheduler.is_running("test_action") is True

    def test_schedule_multiple_actions(self) -> None:
        """
        Tests scheduling multiple actions concurrently.
        """
        scheduler = ActionScheduler()
        action1 = create_mock_action(send_count=2)
        action2 = create_mock_action(send_count=2)
        action3 = create_mock_action(send_count=2)

        scheduler.schedule(action1, "action_1")
        scheduler.schedule(action2, "action_2")
        scheduler.schedule(action3, "action_3")

        assert len(scheduler.get_running_actions()) == 3
        assert scheduler.is_running("action_1") is True
        assert scheduler.is_running("action_2") is True
        assert scheduler.is_running("action_3") is True

    def test_schedule_duplicate_name_returns_none(self) -> None:
        """
        Tests that scheduling an action with a duplicate name returns None.
        """
        scheduler = ActionScheduler()
        action1 = create_mock_action(send_count=2)
        action2 = create_mock_action(send_count=2)

        result1 = scheduler.schedule(action1, "duplicate")
        result2 = scheduler.schedule(action2, "duplicate")

        assert result1 == "duplicate"
        assert result2 is None
        assert len(scheduler.get_running_actions()) == 1

    def test_schedule_different_names_succeeds(self) -> None:
        """
        Tests that different names always succeed in scheduling.
        """
        scheduler = ActionScheduler()
        action1 = create_mock_action(send_count=1)
        action2 = create_mock_action(send_count=1)

        result1 = scheduler.schedule(action1, "name_1")
        result2 = scheduler.schedule(action2, "name_2")

        assert result1 == "name_1"
        assert result2 == "name_2"


class TestActionSchedulerExecution:
    """
    Tests for running scheduled actions.
    """
    def test_run_executes_scheduled_actions(self) -> None:
        """
        Tests that run() executes all scheduled actions once.
        """
        scheduler = ActionScheduler()
        action = create_mock_action(send_count=1)

        scheduler.schedule(action, "test_action")
        scheduler.run()

        assert scheduler.is_running("test_action") is False

    def test_run_removes_completed_actions(self) -> None:
        """
        Tests that run() removes actions that complete.
        """
        scheduler = ActionScheduler()
        action = create_mock_action(send_count=1)

        scheduler.schedule(action, "test_action")
        assert len(scheduler.get_running_actions()) == 1

        scheduler.run()

        assert len(scheduler.get_running_actions()) == 0

    def test_run_continues_incomplete_actions(self) -> None:
        """
        Tests that run() continues actions that need more iterations.
        """
        scheduler = ActionScheduler()
        action = create_mock_action(send_count=3)

        scheduler.schedule(action, "test_action")
        scheduler.run()

        assert scheduler.is_running("test_action") is True
        scheduler.run()
        assert scheduler.is_running("test_action") is True
        scheduler.run()
        assert scheduler.is_running("test_action") is False

    def test_run_multiple_actions_concurrently(self) -> None:
        """
        Tests that run() executes multiple actions concurrently.
        """
        scheduler = ActionScheduler()
        action1 = create_mock_action(send_count=2)
        action2 = create_mock_action(send_count=3)

        scheduler.schedule(action1, "action_1")
        scheduler.schedule(action2, "action_2")

        scheduler.run()

        assert scheduler.is_running("action_1") is True
        assert scheduler.is_running("action_2") is True

        scheduler.run()

        assert scheduler.is_running("action_1") is False
        assert scheduler.is_running("action_2") is True

    def test_run_on_empty_scheduler(self) -> None:
        """
        Tests running an empty scheduler does not raise errors.
        """
        scheduler = ActionScheduler()
        scheduler.run()
        assert len(scheduler.get_running_actions()) == 0


class TestActionSchedulerCancellation:
    """
    Tests for cancelling scheduled actions.
    """
    def test_cancel_running_action(self) -> None:
        """
        Tests cancelling a currently running action.
        """
        scheduler = ActionScheduler()
        action = create_infinite_action()

        scheduler.schedule(action, "test_action")
        assert scheduler.is_running("test_action") is True

        result = scheduler.cancel("test_action")

        assert result == "test_action"
        assert scheduler.is_running("test_action") is False

    def test_cancel_nonexistent_action(self) -> None:
        """
        Tests cancelling an action that does not exist.
        """
        scheduler = ActionScheduler()
        result = scheduler.cancel("nonexistent")
        assert result is None

    def test_cancel_completed_action(self) -> None:
        """
        Tests cancelling an action that already completed.
        """
        scheduler = ActionScheduler()
        action = create_mock_action(send_count=1)

        scheduler.schedule(action, "test_action")
        scheduler.run()

        result = scheduler.cancel("test_action")

        assert result is None

    def test_cancel_removes_from_actions(self) -> None:
        """
        Tests that cancel removes the action from internal state.
        """
        scheduler = ActionScheduler()
        action = create_infinite_action()

        scheduler.schedule(action, "test_action")
        scheduler.cancel("test_action")

        assert len(scheduler._actions) == 0


class TestActionSchedulerCancelAll:
    """
    Tests for cancelling all scheduled actions.
    """
    def test_cancel_all_with_multiple_actions(self) -> None:
        """
        Tests cancelling all actions when multiple are running.
        """
        scheduler = ActionScheduler()
        action1 = create_infinite_action()
        action2 = create_infinite_action()
        action3 = create_infinite_action()

        scheduler.schedule(action1, "action_1")
        scheduler.schedule(action2, "action_2")
        scheduler.schedule(action3, "action_3")

        result = scheduler.cancel_all()

        assert result == ["action_1", "action_2", "action_3"]
        assert len(scheduler.get_running_actions()) == 0

    def test_cancel_all_with_no_actions(self) -> None:
        """
        Tests cancelling all when no actions are running.
        """
        scheduler = ActionScheduler()
        result = scheduler.cancel_all()
        assert result is None

    def test_cancel_all_returns_cancelled_names(self) -> None:
        """
        Tests that cancel_all returns list of cancelled action names.
        """
        scheduler = ActionScheduler()
        action1 = create_infinite_action()
        action2 = create_infinite_action()

        scheduler.schedule(action1, "first")
        scheduler.schedule(action2, "second")

        result = scheduler.cancel_all()

        assert result is not None
        assert len(result) == 2
        assert "first" in result
        assert "second" in result

    def test_cancel_all_clears_actions(self) -> None:
        """
        Tests that cancel_all completely clears the actions dict.
        """
        scheduler = ActionScheduler()
        action1 = create_infinite_action()
        action2 = create_infinite_action()

        scheduler.schedule(action1, "action_1")
        scheduler.schedule(action2, "action_2")

        scheduler.cancel_all()

        assert scheduler._actions == {}


class TestActionSchedulerStateQueries:
    """
    Tests for querying the state of the scheduler.
    """
    def test_is_running_true_for_active_action(self) -> None:
        """
        Tests is_running returns True for an active action.
        """
        scheduler = ActionScheduler()
        action = create_infinite_action()

        scheduler.schedule(action, "test_action")

        assert scheduler.is_running("test_action") is True

    def test_is_running_false_for_inactive_action(self) -> None:
        """
        Tests is_running returns False for non-existent action.
        """
        scheduler = ActionScheduler()
        assert scheduler.is_running("test_action") is False

    def test_is_running_false_after_completion(self) -> None:
        """
        Tests is_running returns False after action completes.
        """
        scheduler = ActionScheduler()
        action = create_mock_action(send_count=1)

        scheduler.schedule(action, "test_action")
        scheduler.run()

        assert scheduler.is_running("test_action") is False

    def test_get_running_actions_returns_list(self) -> None:
        """
        Tests get_running_actions returns a list of current actions.
        """
        scheduler = ActionScheduler()
        action1 = create_infinite_action()
        action2 = create_infinite_action()

        scheduler.schedule(action1, "action_1")
        scheduler.schedule(action2, "action_2")

        actions = scheduler.get_running_actions()

        assert isinstance(actions, list)
        assert len(actions) == 2

    def test_get_running_actions_empty_scheduler(self) -> None:
        """
        Tests get_running_actions on an empty scheduler.
        """
        scheduler = ActionScheduler()
        actions = scheduler.get_running_actions()
        assert actions == []


class TestActionSchedulerComplexScenarios:
    """
    Tests for complex multi-step scheduling scenarios.
    """
    def test_schedule_run_and_cancel_sequence(self) -> None:
        """
        Tests scheduling, running, and cancelling in sequence.
        """
        scheduler = ActionScheduler()
        action1 = create_mock_action(send_count=2)
        action2 = create_infinite_action()

        scheduler.schedule(action1, "short_action")
        scheduler.schedule(action2, "long_action")

        scheduler.run()

        assert scheduler.is_running("short_action") is True
        assert scheduler.is_running("long_action") is True

        scheduler.cancel("long_action")

        assert scheduler.is_running("short_action") is True
        assert scheduler.is_running("long_action") is False

        scheduler.run()

        assert scheduler.is_running("short_action") is False

    def test_immediate_reschedule_after_completion(self) -> None:
        """
        Tests rescheduling an action name after previous one completes.
        """
        scheduler = ActionScheduler()
        action1 = create_mock_action(send_count=1)

        scheduler.schedule(action1, "reusable")
        scheduler.run()

        action2 = create_mock_action(send_count=1)
        result = scheduler.schedule(action2, "reusable")

        assert result == "reusable"
        assert scheduler.is_running("reusable") is True

    def test_mixed_completion_and_cancellation(self) -> None:
        """
        Tests a mix of actions completing and being cancelled.
        """
        scheduler = ActionScheduler()
        action1 = create_mock_action(send_count=1)
        action2 = create_mock_action(send_count=3)
        action3 = create_infinite_action()

        scheduler.schedule(action1, "completes_first")
        scheduler.schedule(action2, "completes_later")
        scheduler.schedule(action3, "cancelled")

        scheduler.run()

        assert scheduler.is_running("completes_first") is False
        assert len(scheduler.get_running_actions()) == 2

        scheduler.cancel("cancelled")

        assert len(scheduler.get_running_actions()) == 1

        scheduler.run()
        assert scheduler.is_running("completes_later") is True

        scheduler.run()
        assert scheduler.is_running("completes_later") is False

    def test_multiple_run_calls_progress_actions(self) -> None:
        """
        Tests that multiple run() calls progressively execute actions.
        """
        scheduler = ActionScheduler()
        action = create_mock_action(send_count=4)

        scheduler.schedule(action, "test_action")

        for i in range(4):
            if i < 3:
                assert scheduler.is_running("test_action") is True
            scheduler.run()

        assert scheduler.is_running("test_action") is False
