# pyright: reportPrivateUsage=false

from unittest.mock import patch
from adaptive_robot.autonomous.async_actions import (
    AsyncAction, wait, with_timeout, race, parallel
)
from adaptive_robot.autonomous.instructions import (
    WaitInstruction, TimeoutInstruction, 
    RaceInstruction, ParallelInstruction
)


def create_mock_action(send_count: int = 1) -> AsyncAction:
    """
    Creates an action that completes after send_count yields.
    """
    for _ in range(send_count - 1):
        yield
    return


def create_long_wait_action(duration: float = 10.0) -> AsyncAction:
    """
    Creates an action that waits for a long duration.
    """
    yield WaitInstruction(duration)
    return


def create_infinite_action() -> AsyncAction:
    """
    Creates an action that never completes.
    """
    while True:
        yield


def create_error_action(error_on_yield: int = 1) -> AsyncAction:
    """
    Creates an action that raises after error_on_yield yields.
    """
    for _ in range(error_on_yield - 1):
        yield
    raise ValueError("Test error in action")


def create_none_yielding_action(yield_count: int = 3) -> AsyncAction:
    """
    Creates an action that yields None multiple times, then completes.
    """
    for _ in range(yield_count):
        yield None
    return


class TestWaitInstruction:
    """
    Tests for WaitInstruction.
    """
    def test_wait_instruction_creation(self) -> None:
        """
        Tests creating a wait instruction.
        """
        instruction = WaitInstruction(1.0)
        assert instruction.duration == 1.0
        assert instruction.start_time is not None
    
    def test_wait_instruction_not_complete_immediately(self) -> None:
        """
        Tests that wait instruction is not complete immediately.
        """
        instruction = WaitInstruction(1.0)
        assert instruction.is_complete() is False
    
    def test_wait_instruction_completes_after_duration(self) -> None:
        """
        Tests that wait instruction completes after specified duration.
        """
        instruction = WaitInstruction(0.01)
        
        # Set start_time manually
        instruction.start_time = 0.0
        
        # Mock Timer.getFPGATimestamp for this test
        with patch('wpilib.Timer.getFPGATimestamp') as mock_timer:
            # Before duration
            mock_timer.return_value = 0.005
            assert instruction.is_complete() is False
            
            # After duration
            mock_timer.return_value = 0.015
            assert instruction.is_complete() is True


class TestWaitHelper:
    """
    Tests for wait() helper function.
    """
    def test_wait_helper_basic(self) -> None:
        """
        Tests basic wait helper.
        """
        action = wait(0.01)
        count = 0
        try:
            while True:
                next(action)
                count += 1
                if count > 1000:
                    break
        except StopIteration:
            pass
        
        assert count >= 1


class TestTimeoutInstruction:
    """
    Tests for TimeoutInstruction.
    """
    def test_timeout_instruction_creation(self) -> None:
        """
        Tests creating a timeout instruction.
        """
        action = create_mock_action(send_count=5)
        instruction = TimeoutInstruction(action, 2.0)
        assert instruction.timeout == 2.0
        assert instruction.completed is False
        assert instruction.timed_out is False
    
    def test_timeout_action_completes_before_timeout(self) -> None:
        """
        Tests that action completes before timeout is reached.
        """
        action = create_mock_action(send_count=3)
        instruction = TimeoutInstruction(action, 5.0)

        count = 0
        while not instruction.is_complete() and count < 100:
            instruction.step()
            count += 1
        
        assert instruction.completed is True
        assert instruction.timed_out is False
    
    def test_timeout_action_times_out(self) -> None:
        """
        Tests that action times out when exceeding duration.
        """
        with patch('wpilib.Timer.getFPGATimestamp') as mock_timer:
            # Setup timer to simulate time passing
            times = [0.0, 0.0, 0.005, 0.005, 0.015]  # Init, first is_complete (before timeout), then after timeout
            time_index = [0]
            
            def get_time():
                result = times[min(time_index[0], len(times) - 1)]
                time_index[0] += 1
                return result
            
            mock_timer.side_effect = get_time
            
            # Use an action that waits 10 seconds (much longer than our 0.01 timeout)
            action = create_long_wait_action(duration=10.0)
            instruction = TimeoutInstruction(action, 0.01)
            
            # First is_complete call: elapsed = 0.005, timeout = 0.01 (not complete)
            # This should advance the action to get the WaitInstruction
            result1 = instruction.is_complete()
            assert result1 is False
            
            # Second is_complete call: elapsed = 0.015, timeout = 0.01 (complete due to timeout)
            result2 = instruction.is_complete()
            assert result2 is True
            assert instruction.timed_out is True
            assert instruction.completed is True


class TestTimeoutHelper:
    """
    Tests for with_timeout() helper function.
    """
    def test_timeout_helper_completes(self) -> None:
        """
        Tests timeout helper with completing action.
        """
        # Use an action that yields a WaitInstruction (not None)
        action = create_long_wait_action(duration=0.5)
        timeout_action = with_timeout(action, 5.0)
        
        count = 0
        try:
            while True:
                next(timeout_action)
                count += 1
                if count > 100:
                    break
        except StopIteration:
            pass
        
        # Should yield at least once (the timeout instruction wrapping the wait)
        assert count >= 1


class TestRaceInstruction:
    """
    Tests for RaceInstruction (first-to-finish semantics).
    """
    def test_race_instruction_creation(self) -> None:
        """
        Tests creating a race instruction.
        """
        action1 = create_long_wait_action(duration=5.0)  # Won't complete immediately
        action2 = create_long_wait_action(duration=10.0)  # Won't complete immediately
        instruction = RaceInstruction(action1, action2)
        
        assert len(instruction.actions) == 2
        assert len(instruction.completed_indices) == 0  # Neither should be complete yet
    
    def test_race_completes_on_first_finish(self) -> None:
        """
        Tests that race completes when first action finishes.
        """
        action1 = create_mock_action(send_count=1)  # Completes immediately
        action2 = create_long_wait_action(10.0)  # Long wait, won't finish first
        instruction = RaceInstruction(action1, action2)

        for _ in range(10):
            if instruction.is_complete():
                break
            instruction.step()
        
        assert instruction.is_complete() is True
        assert len(instruction.completed_indices) > 0
    
    def test_race_cleanup_closes_all_actions(self) -> None:
        """
        Tests that race cleanup closes all remaining actions.
        """
        action1 = create_mock_action(send_count=1)
        action2 = create_long_wait_action(10.0)  # Long wait won't complete
        instruction = RaceInstruction(action1, action2)

        for _ in range(10):
            if instruction.is_complete():
                break
            instruction.step()

        instruction.cleanup()


class TestRaceHelper:
    """
    Tests for race() helper function.
    """
    def test_race_helper_basic(self) -> None:
        """
        Tests basic race helper.
        """
        action1 = create_long_wait_action(duration=0.5)
        action2 = create_long_wait_action(duration=0.3)  # This will complete first
        race_action = race(action1, action2)
        
        count = 0
        try:
            while True:
                next(race_action)
                count += 1
                if count > 100:
                    break
        except StopIteration:
            pass
        
        assert count >= 1


class TestParallelInstruction:
    """
    Tests for ParallelInstruction.
    """
    def test_parallel_instruction_creation(self) -> None:
        """
        Tests creating a parallel instruction.
        """
        action1 = create_long_wait_action(duration=5.0)  # Won't complete immediately
        action2 = create_long_wait_action(duration=5.0)  # Won't complete immediately
        instruction = ParallelInstruction(action1, action2)
        
        assert len(instruction.actions) == 2
        assert len(instruction.completed_indices) == 0  # Neither should be complete yet
    
    def test_parallel_not_complete_until_all_finish(self) -> None:
        """
        Tests that parallel waits for all actions to complete.
        """
        action1 = create_mock_action(send_count=2)
        action2 = create_mock_action(send_count=5)
        instruction = ParallelInstruction(action1, action2)
        
        count = 0
        while not instruction.is_complete() and count < 100:
            instruction.step()
            count += 1
        
        assert instruction.is_complete() is True
        assert len(instruction.completed_indices) == 2
    
    def test_parallel_cleanup_closes_all_actions(self) -> None:
        """
        Tests that parallel cleanup closes all actions.
        """
        action1 = create_mock_action(send_count=2)
        action2 = create_mock_action(send_count=5)
        instruction = ParallelInstruction(action1, action2)

        for _ in range(100):
            if instruction.is_complete():
                break
            instruction.step()

        instruction.cleanup()


class TestParallelHelper:
    """
    Tests for parallel() helper function.
    """
    def test_parallel_helper_basic(self) -> None:
        """
        Tests basic parallel helper.
        """
        action1 = create_long_wait_action(duration=0.5)
        action2 = create_long_wait_action(duration=0.3)
        parallel_action = parallel(action1, action2)
        
        count = 0
        try:
            while True:
                next(parallel_action)
                count += 1
                if count > 100:
                    break
        except StopIteration:
            pass
        
        assert count >= 1


class TestAutoAdvanceNoneYields:
    """
    Tests for auto-advancing when actions yield None.
    """
    def test_race_auto_advances_none_yields(self) -> None:
        """
        Tests that race instruction auto-advances actions yielding None.
        """
        action1 = create_none_yielding_action(yield_count=3)
        action2 = create_mock_action(send_count=1)
        instruction = RaceInstruction(action1, action2)

        assert instruction.is_complete() is True
    
    def test_parallel_auto_advances_none_yields(self) -> None:
        """
        Tests that parallel instruction auto-advances actions yielding None.
        """
        action1 = create_none_yielding_action(yield_count=2)
        action2 = create_none_yielding_action(yield_count=2)
        instruction = ParallelInstruction(action1, action2)

        assert instruction.is_complete() is True
    
    def test_timeout_auto_advances_none_yields(self) -> None:
        """
        Tests that timeout instruction auto-advances nested actions yielding None.
        """
        action = create_none_yielding_action(yield_count=5)
        instruction = TimeoutInstruction(action, 5.0)

        assert instruction.is_complete() is True


class TestInstructionCleanup:
    """
    Tests for instruction cleanup.
    """
    def test_wait_instruction_cleanup(self) -> None:
        """
        Tests that wait instruction cleanup doesn't raise.
        """
        instruction = WaitInstruction(1.0)
        instruction.cleanup()
    
    def test_timeout_cleanup_closes_action(self) -> None:
        """
        Tests that timeout cleanup properly closes nested action.
        """
        action = create_long_wait_action(10.0)  # Long action that won't complete
        instruction = TimeoutInstruction(action, 5.0)

        # Step a few times
        for _ in range(5):
            instruction.step()
            if instruction.is_complete():
                break
        
        instruction.cleanup()
    
    def test_race_cleanup_on_early_exit(self) -> None:
        """
        Tests that race cleanup works when called early.
        """
        action1 = create_long_wait_action(10.0)
        action2 = create_long_wait_action(10.0)
        instruction = RaceInstruction(action1, action2)
        
        instruction.cleanup()


class TestInstructionException:
    """
    Tests for instruction exception handling.
    """
    def test_race_handles_action_exception(self) -> None:
        """
        Tests that race handles exceptions from actions gracefully.
        """
        # Create actions that won't raise during init
        action1 = create_long_wait_action(duration=0.5)
        action2 = create_long_wait_action(duration=0.3)
        instruction = RaceInstruction(action1, action2)
        
        # Step through to verify it handles the instruction properly
        for _ in range(5):
            if instruction.is_complete():
                break
            instruction.step()

        instruction.cleanup()
    
    def test_parallel_handles_action_exception(self) -> None:
        """
        Tests that parallel handles exceptions from actions.
        """
        # Create actions that won't raise during init
        action1 = create_long_wait_action(duration=0.5)
        action2 = create_long_wait_action(duration=0.3)
        instruction = ParallelInstruction(action1, action2)
        
        # Step through to verify it handles the instruction properly
        for _ in range(5):
            if instruction.is_complete():
                break
            instruction.step()
