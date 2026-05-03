from adaptive_robot.requests import RequestArbitrator


class TestRequestArbitratorInitialization:
    """
    Tests for RequestArbitrator initialization.
    """
    def test_default_initialization(self) -> None:
        """
        Tests default RequestArbitrator creation.
        """
        controller = RequestArbitrator()
        default = controller.get_default_request()
        assert default.value == 0.0
        assert default.priority == -1
        assert default.source == 'default'
        assert controller.is_enabled() is True
        assert controller.last_request == default
    
    def test_initialization_with_custom_defaults(self) -> None:
        """
        Tests RequestArbitrator with custom defaults.
        """
        controller = RequestArbitrator(
            default_value=0.5,
            default_priority=5,
            default_source='custom'
        )
        default = controller.get_default_request()
        assert default.value == 0.5
        assert default.priority == 5
        assert default.source == 'custom'
    
    def test_last_request_property(self) -> None:
        """
        Tests accessing last_request property.
        """
        controller = RequestArbitrator()
        assert controller.last_request == controller.get_default_request()


class TestRequestArbitratorRequests:
    """
    Tests for request submission and management.
    """
    def test_single_request(self) -> None:
        """
        Tests submitting a single request.
        """
        controller = RequestArbitrator()
        controller.request(value=0.5, priority=10, source="motor1")
        
        resolved = controller.resolve()
        assert resolved.value == 0.5
        assert resolved.priority == 10
        assert resolved.source == "motor1"
    
    def test_multiple_requests_priority(self) -> None:
        """
        Tests that highest priority request wins.
        """
        controller = RequestArbitrator()
        controller.request(value=0.3, priority=5, source="low")
        controller.request(value=0.8, priority=15, source="high")
        controller.request(value=0.5, priority=10, source="medium")
        
        resolved = controller.resolve()
        assert resolved.source == "high"
        assert resolved.value == 0.8
    
    def test_same_priority_uses_latest(self) -> None:
        """
        Tests that with same priority, most recent request wins.
        """
        controller = RequestArbitrator()
        controller.request(value=0.3, priority=10, source="first")
        controller.request(value=0.8, priority=10, source="second")
        
        resolved = controller.resolve()
        assert resolved.source == "second"
        assert resolved.value == 0.8
    
    def test_clear_requests(self) -> None:
        """
        Tests clearing all requests.
        """
        controller = RequestArbitrator()
        controller.request(value=0.5, priority=10, source="test")
        controller.clear()
        
        resolved = controller.resolve()
        assert resolved == controller.get_default_request()
    
    def test_overwrite_source_request(self) -> None:
        """
        Tests that same source overwrites previous request.
        """
        controller = RequestArbitrator()
        controller.request(value=0.3, priority=10, source="motor")
        controller.request(value=0.8, priority=10, source="motor")
        
        resolved = controller.resolve()
        assert resolved.value == 0.8


class TestRequestArbitratorEnabled:
    """
    Tests for enabled/disabled state.
    """
    def test_disabled_clears_requests_on_new_request(self) -> None:
        """
        Tests that requests are cleared when request() is called while disabled.
        """
        controller = RequestArbitrator()
        controller.request(value=0.5, priority=10, source="test")

        controller.set_enabled(False)
        controller.request(value=0.3, priority=5, source="new")
        
        resolved = controller.resolve()
        assert resolved == controller.get_default_request()
        assert controller.get_pending_request_count() == 0
    
    def test_disabled_ignores_new_requests(self) -> None:
        """
        Tests that new requests are ignored when disabled.
        """
        controller = RequestArbitrator()
        controller.set_enabled(False)
        controller.request(value=0.5, priority=10, source="test")
        
        assert controller.get_pending_request_count() == 0
    
    def test_re_enabled_accepts_requests(self):
        """
        Tests that re-enabling accepts requests again.
        """
        controller = RequestArbitrator()
        controller.set_enabled(False)
        controller.set_enabled(True)
        
        controller.request(value=0.5, priority=10, source="test")
        resolved = controller.resolve()
        assert resolved.value == 0.5


class TestRequestArbitratorLastRequest:
    """
    Tests for last_request tracking.
    """
    def test_last_request_persistent(self) -> None:
        """
        Tests that last_request persists after resolve.
        """
        controller = RequestArbitrator()
        controller.request(value=0.5, priority=10, source="test")
        
        resolved = controller.resolve()
        assert controller.last_request == resolved
    
    def test_last_request_when_default(self) -> None:
        """
        Tests last_request when using default.
        """
        controller = RequestArbitrator()
        _ = controller.resolve()
        
        assert controller.last_request == controller.get_default_request()


class TestRequestArbitratorComplexScenarios:
    """
    Tests for complex scenarios and edge cases.
    """
    def test_multiple_sources_different_priorities(self) -> None:
        """
        Tests arbitration with multiple sources and priorities.
        """
        controller = RequestArbitrator()

        controller.request(value=0.2, priority=1, source="drivetrain")
        controller.request(value=-0.8, priority=20, source="arm")
        controller.request(value=0.5, priority=10, source="intake")
        
        resolved = controller.resolve()
        assert resolved.source == "arm"
        assert resolved.value == -0.8
    
    def test_multiple_requests_priority_based_resolution(self) -> None:
        """
        Tests that multiple requests are resolved by priority.
        """
        controller = RequestArbitrator()
        
        controller.request(value=0.8, priority=20, source="high_priority")
        controller.request(value=0.3, priority=5, source="low_priority")
        controller.request(value=0.6, priority=15, source="medium_priority")
        
        resolved = controller.resolve()
        assert resolved.source == "high_priority"
        assert resolved.value == 0.8
