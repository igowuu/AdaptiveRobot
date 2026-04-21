import time
import math
import pytest

from adaptive_robot.requests import AxisController
from adaptive_robot.adaptive_component.adaptive_component import AdaptiveComponent


class TestAdversarialAxisController:
    """
    Tests designed to break AxisController.
    """
    def test_concurrent_request_and_clear(self) -> None:
        """
        Try to cause race conditions by mixing requests and clears.
        """
        controller = AxisController()
        
        for i in range(50):
            if i % 2 == 0:
                controller.request(value=0.5, priority=10, source=f"req_{i}")
            else:
                controller.clear()
            
            resolved = controller.resolve()
            assert resolved is not None
    
    def test_extreme_priority_inversion(self) -> None:
        """
        Tests very large priority differences.
        """
        controller = AxisController()

        controller.request(value=0.5, priority=-2147483648, source="lowest")  # Min 32-bit int
        controller.request(value=0.8, priority=0, source="zero")
        controller.request(value=0.3, priority=2147483647, source="highest")  # Max 32-bit int
        
        resolved = controller.resolve()
        assert resolved.source == "highest"
    
    def test_maximum_nested_requests(self) -> None:
        """
        Tests with a huge number of sources all at once.
        """
        controller = AxisController()

        for i in range(10000):
            controller.request(
                value=float(i % 100) / 100.0,
                priority=i % 100,
                source=f"source_{i}"
            )

        resolved = controller.resolve()
        assert resolved is not None
        assert resolved.priority == 99
    
    def test_request_with_nan_values(self) -> None:
        """
        Tests requesting with NaN values.
        """
        controller = AxisController()

        nan_value = float('nan')
        controller.request(value=nan_value, priority=10, source="nan")
        
        resolved = controller.resolve()
        assert resolved.source == "nan"
        assert math.isnan(resolved.value) or resolved.value != resolved.value


class TestAdversarialTimeouts:
    """
    Tests designed to break timeout handling.
    """
    def test_request_timeout_during_resolve(self) -> None:
        """
        Tests request timing out between request and resolve.
        """
        controller = AxisController()
        
        controller.request(value=0.5, priority=10, source="ephemeral", timeout=0.001)

        resolved = controller.resolve()
        assert resolved.source == "ephemeral"

        time.sleep(0.002)

        resolved = controller.resolve()
        assert resolved == controller.get_default_request()
    
    def test_request_timeout_edge_at_zero(self) -> None:
        """
        Tests request with zero timeout.
        """
        controller = AxisController()
        
        # Zero timeout should raise
        with pytest.raises(ValueError):
            controller.request(value=0.5, priority=10, source="zero_timeout", timeout=0.0)
    
    def test_mixed_timeout_and_priority(self) -> None:
        """
        Tests interactions between timeout and priority logic.
        """
        controller = AxisController()

        controller.request(value=0.9, priority=100, source="timeout_high", timeout=0.005)
  
        controller.request(value=0.1, priority=1, source="persistent_low", timeout=10.0)

        resolved = controller.resolve()
        assert resolved.source == "timeout_high"

        time.sleep(0.01)

        resolved = controller.resolve()
        assert resolved.source == "persistent_low"


class TestAdversarialComponentContext:
    """
    Tests for AdaptiveComponent with edge case contexts.
    """
    def test_component_health_flip_flop(self) -> None:
        """
        Tests rapidly changing component health.
        """
        class TestComponent(AdaptiveComponent):
            def execute(self) -> None:
                pass
        
        component = TestComponent()

        for i in range(100):
            component.set_health(i % 2 == 0)
            assert component.is_healthy() == (i % 2 == 0)


class TestAdversarialMemoryBehavior:
    """
    Tests for potential memory or resource leaks.
    """
    def test_massive_request_accumulation(self) -> None:
        """
        Tests accumulating huge numbers of requests.
        """
        controller = AxisController()

        for i in range(100000):
            controller.request(
                value=float(i % 100) / 100.0,
                priority=i % 10,
                source=f"massive_{i}"
            )

        resolved = controller.resolve()
        assert resolved is not None

        controller.clear()
        assert controller.get_pending_request_count() == 0
    
    def test_request_with_very_long_source_names(self) -> None:
        """
        Tests with extremely long source names.
        """
        controller = AxisController()

        long_source = "x" * 1000

        with pytest.raises(ValueError):
            controller.request(value=0.5, priority=10, source=long_source)


class TestAdversarialPriorityHandling:
    """
    Tests priority handling.
    """
    def test_all_requests_same_source_different_priorities(self) -> None:
        """
        Tests updating same source with different priorities.
        """
        controller = AxisController()

        for i in range(100):
            controller.request(value=0.5, priority=i, source="variable")
        
        resolved = controller.resolve()
        assert resolved.priority == 99
        assert resolved.source == "variable"
    
    def test_timestamp_tiebreaker_accuracy(self) -> None:
        """
        Tests that timestamp tiebreaker works correctly.
        """
        controller = AxisController()

        controller.request(value=0.1, priority=10, source="first")
        first_timestamp = controller._requests["first"].timestamp # type: ignore
        
        controller.request(value=0.2, priority=10, source="second")
        second_timestamp = controller._requests["second"].timestamp # type: ignore
        
        resolved = controller.resolve()

        assert resolved.source == "second"
        assert second_timestamp >= first_timestamp
