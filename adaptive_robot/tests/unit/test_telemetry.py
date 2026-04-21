# pyright: reportPrivateUsage=false

from unittest.mock import Mock, patch, MagicMock
from adaptive_robot.telemetry.telemetry import TelemetryPublisher
from adaptive_robot.telemetry.struct_telemetry import TelemetryStructPublisher, StructEntry


class TestTelemetryPublisherBasic:
    """
    Tests for basic TelemetryPublisher functionality.
    """
    def test_initialization(self) -> None:
        """
        Tests TelemetryPublisher initialization.
        """
        publisher = TelemetryPublisher()
        assert publisher.nt is not None
        assert publisher._last_values == {}
    
    def test_put_value_boolean(self) -> None:
        """
        Tests publishing boolean values.
        """
        publisher = TelemetryPublisher()
        publisher.put_value("test/bool", True)
        
        assert publisher._last_values["test/bool"] is True
    
    def test_put_value_integer(self) -> None:
        """
        Tests publishing integer values.
        """
        publisher = TelemetryPublisher()
        publisher.put_value("test/int", 42)
        
        assert publisher._last_values["test/int"] == 42
    
    def test_put_value_float(self) -> None:
        """
        Tests publishing float values.
        """
        publisher = TelemetryPublisher()
        publisher.put_value("test/float", 3.14159)
        
        # Should be rounded to 5 digits
        assert publisher._last_values["test/float"] == 3.14159
    
    def test_put_value_string(self) -> None:
        """
        Tests publishing string values.
        """
        publisher = TelemetryPublisher()
        publisher.put_value("test/string", "hello")
        
        assert publisher._last_values["test/string"] == "hello"


class TestTelemetryPublisherChangeDetection:
    """
    Tests for change detection in TelemetryPublisher.
    """
    def test_no_republish_same_value(self) -> None:
        """
        Tests that same value isn't republished.
        """
        publisher = TelemetryPublisher()
        
        # Mock the _publish method to track calls
        with patch.object(publisher, '_publish') as mock_publish:
            publisher.put_value("test/float", 0.5)
            assert mock_publish.call_count == 1
            
            # Second call with same value should skip
            publisher.put_value("test/float", 0.5)
            assert mock_publish.call_count == 1  # Still 1
    
    def test_republish_different_value(self) -> None:
        """
        Tests that different value is republished.
        """
        publisher = TelemetryPublisher()
        
        with patch.object(publisher, '_publish') as mock_publish:
            publisher.put_value("test/float", 0.5)
            publisher.put_value("test/float", 0.8)
            assert mock_publish.call_count == 2
    
    def test_republish_after_change(self) -> None:
        """
        Tests republishing after value changes.
        """
        publisher = TelemetryPublisher()
        
        with patch.object(publisher, '_publish') as mock_publish:
            publisher.put_value("key", True)
            assert mock_publish.call_count == 1
            
            publisher.put_value("key", False)
            assert mock_publish.call_count == 2
            
            publisher.put_value("key", False)
            assert mock_publish.call_count == 2  # No change


class TestTelemetryPublisherRounding:
    """
    Tests for float rounding.
    """
    def test_float_rounding_default(self) -> None:
        """
        Tests default float rounding to 5 digits.
        """
        publisher = TelemetryPublisher()
        
        # Value with many decimal places
        long_value = 3.14159265358979
        publisher.put_value("test", long_value)
        
        stored = publisher._last_values["test"]
        assert stored == round(long_value, 5)
    
    def test_integer_not_rounded(self) -> None:
        """
        Tests that integers are not rounded.
        """
        publisher = TelemetryPublisher()
        publisher.put_value("test", 42)
        
        assert publisher._last_values["test"] == 42
    
    def test_string_not_affected(self) -> None:
        """
        Tests that strings are not affected by rounding.
        """
        publisher = TelemetryPublisher()
        publisher.put_value("test", "3.14159265358979")
        
        assert publisher._last_values["test"] == "3.14159265358979"


class TestTelemetryPublisherGetValue:
    """
    Tests for retrieving values.
    """
    def test_get_existing_value(self) -> None:
        """
        Tests getting a value that exists.
        """
        publisher = TelemetryPublisher()
        publisher.put_value("test/float", 2.5)
        
        value = publisher.get_value("test/float")
        assert value == 2.5
    
    def test_get_nonexistent_value(self) -> None:
        """
        Tests getting a value that doesn't exist.
        """
        publisher = TelemetryPublisher()
        
        value = publisher.get_value("nonexistent")
        # Should return the value from NetworkTables entry (which returns None for missing values)
        assert value is None or isinstance(value, (int, float, str, bool))


class TestTelemetryPublisherTypeHandling:
    """
    Tests for different type handling.
    """
    def test_type_preservation(self) -> None:
        """
        Tests that types are preserved.
        """
        publisher = TelemetryPublisher()
        
        # Boolean
        publisher.put_value("bool_key", True)
        assert isinstance(publisher._last_values["bool_key"], bool)
        
        # Integer
        publisher.put_value("int_key", 42)
        assert isinstance(publisher._last_values["int_key"], int)
        
        # Float
        publisher.put_value("float_key", 3.14)
        assert isinstance(publisher._last_values["float_key"], float)
        
        # String
        publisher.put_value("str_key", "test")
        assert isinstance(publisher._last_values["str_key"], str)


class TestTelemetryStructPublisherBasic:
    """
    Tests for basic TelemetryStructPublisher functionality.
    """
    def test_initialization(self) -> None:
        """
        Tests TelemetryStructPublisher initialization.
        """
        publisher = TelemetryStructPublisher()
        assert publisher._entries == {}
    
    @patch('adaptive_robot.telemetry.struct_telemetry.NetworkTableInstance')
    def test_put_struct_value_first_time(self, mock_nt_instance: Mock) -> None:
        """
        Tests publishing a struct value for the first time.
        """
        mock_topic = MagicMock()
        mock_publisher_obj = MagicMock()
        mock_subscriber_obj = MagicMock()
        
        mock_topic.publish.return_value = mock_publisher_obj
        mock_publisher_obj.getTopic.return_value = mock_topic
        mock_topic.subscribe.return_value = mock_subscriber_obj
        
        mock_instance = MagicMock()
        mock_instance.getStructTopic.return_value = mock_topic
        mock_nt_instance.getDefault.return_value = mock_instance
        
        publisher = TelemetryStructPublisher()
        
        mock_value = MagicMock()
        publisher.put_struct_value("test/pose", mock_value)
        
        assert "test/pose" in publisher._entries
        entry = publisher._entries["test/pose"]
        assert entry.cached_value == mock_value


class TestTelemetryStructPublisherMultipleValues:
    """
    Tests for handling multiple struct values.
    """
    @patch('adaptive_robot.telemetry.struct_telemetry.NetworkTableInstance')
    def test_multiple_struct_values(self, mock_nt_instance: Mock) -> None:
        """
        Tests publishing multiple different struct values.
        """
        # Setup mock returns for struct publishing
        mock_topic = MagicMock()
        mock_publisher_obj = MagicMock()
        mock_subscriber_obj = MagicMock()
        
        mock_topic.publish.return_value = mock_publisher_obj
        mock_publisher_obj.getTopic.return_value = mock_topic
        mock_topic.subscribe.return_value = mock_subscriber_obj
        
        mock_instance = MagicMock()
        mock_instance.getStructTopic.return_value = mock_topic
        mock_nt_instance.getDefault.return_value = mock_instance
        
        publisher = TelemetryStructPublisher()
        
        value1 = MagicMock()
        value2 = MagicMock()
        
        publisher.put_struct_value("robot/pose", value1)
        publisher.put_struct_value("arm/position", value2)
        
        assert len(publisher._entries) == 2
        assert "robot/pose" in publisher._entries
        assert "arm/position" in publisher._entries


class TestStructEntry:
    """
    Tests for StructEntry dataclass.
    """
    def test_struct_entry_creation(self) -> None:
        """
        Tests creating a StructEntry.
        """
        publisher = MagicMock()
        subscriber = MagicMock()
        cached_value = MagicMock()
        
        entry = StructEntry(publisher, subscriber, cached_value)
        
        assert entry.publisher == publisher
        assert entry.subscriber == subscriber
        assert entry.cached_value == cached_value


class TestTelemetryIntegration:
    """
    Integration tests for telemetry publishing.
    """
    def test_multiple_publishers_same_key(self) -> None:
        """
        Tests that different publishers can publish to same key.
        """
        pub1 = TelemetryPublisher()
        pub2 = TelemetryPublisher()
        
        pub1.put_value("shared/value", 1.0)
        pub2.put_value("shared/value", 2.0)
        
        assert pub1._last_values["shared/value"] == 1.0
        assert pub2._last_values["shared/value"] == 2.0
    
    def test_rapid_value_changes(self) -> None:
        """
        Tests handling rapid value changes.
        """
        publisher = TelemetryPublisher()
        
        with patch.object(publisher, '_publish') as mock_publish:
            for i in range(10):
                publisher.put_value("test", float(i))
            
            # Should have called _publish 10 times (all different values)
            assert mock_publish.call_count == 10
    
    def test_mixed_types_different_keys(self) -> None:
        """
        Tests publishing different types to different keys.
        """
        publisher = TelemetryPublisher()
        
        publisher.put_value("bool", True)
        publisher.put_value("int", 42)
        publisher.put_value("float", 3.14)
        publisher.put_value("string", "test")
        
        assert len(publisher._last_values) == 4
        assert all(key in publisher._last_values for key in 
                  ["bool", "int", "float", "string"])
