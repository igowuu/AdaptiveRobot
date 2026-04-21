import pytest
from unittest.mock import Mock

from adaptive_robot.telemetry.telemetry import primitive_type


class MockNetworkTableEntry:
    """
    Mock NetworkTable entry for testing.
    """
    def __init__(self, default_value: primitive_type = 0) -> None:
        self._value = default_value
        self._type = type(default_value)
    
    def setBoolean(self, value: bool) -> None:
        """
        Sets the NT entry value to a specified boolean.
        """
        self._value = value
    
    def getBoolean(self, default: bool = False) -> bool:
        """
        Returns the value in the entry if it is a boolean, else default.
        """
        return self._value if isinstance(self._value, bool) else default
    
    def setInteger(self, value: int) -> None:
        """
        Sets the NT entry value to a specified int.
        """
        self._value = value
    
    def getInteger(self, default: int = 0) -> int:
        """
        Returns the value in the entry if it is a integer, else default.
        """
        return self._value if isinstance(self._value, int) else default
    
    def setDouble(self, value: float) -> None:
        """
        Sets the NT entry value to a specified float.
        """
        self._value = value
    
    def getDouble(self, default: float = 0.0) -> float:
        """
        Returns the value in the entry if it is a float, else default.
        """
        return self._value if isinstance(self._value, (int, float)) else default
    
    def setString(self, value: str) -> None:
        """
        Sets the NT entry value to a specified string.
        """
        self._value = value
    
    def getString(self, default: str = "") -> str:
        """
        Returns the value in the entry if it is a string, else default.
        """
        return str(self._value) if self._value else default

    def getValue(self) -> primitive_type:
        """
        Returns the value in the entry, no matter the type.
        """
        return self._value


class MockNetworkTable:
    """
    Mock NetworkTable for testing.
    """
    def __init__(self, name: str = "Dashboard") -> None:
        self.name = name
        self._entries: dict[str, MockNetworkTableEntry] = {}
    
    def getEntry(self, key: str) -> MockNetworkTableEntry:
        """
        Returns the networktable entry at a specified key.  
        If it does not exist, creates an empty networktable entry at the specified key
        with a default value of 0.
        """
        if key not in self._entries:
            self._entries[key] = MockNetworkTableEntry()
        return self._entries[key]
    
    def putBoolean(self, key: str, value: bool) -> None:
        """
        Publishes a boolean value to an entry with a specified key.
        """
        entry = self.getEntry(key)
        entry.setBoolean(value)
    
    def putInteger(self, key: str, value: int) -> None:
        """
        Publishes an integer value to an entry with a specified key.
        """
        entry = self.getEntry(key)
        entry.setInteger(value)
    
    def putNumber(self, key: str, value: float) -> None:
        """
        Publishes a float value to an entry with a specified key.
        """
        entry = self.getEntry(key)
        entry.setDouble(value)
    
    def putString(self, key: str, value: str) -> None:
        """
        Publishes a string value to an entry with a specified key.
        """
        entry = self.getEntry(key)
        entry.setString(value)

    def getStructTopic(self, key: str, value_type: type) -> Mock:
        """
        Return a mock struct topic.
        """
        mock_topic = Mock()
        mock_publisher = Mock()
        mock_subscriber = Mock()
        
        mock_publisher.getTopic.return_value = mock_topic
        mock_topic.subscribe.return_value = mock_subscriber
        mock_publisher.set = Mock()
        mock_subscriber.get.return_value = None
        
        mock_topic.publish.return_value = mock_publisher
        return mock_topic


class MockNetworkTableInstance:
    """
    Mock NetworkTableInstance singleton.
    """
    instance: MockNetworkTableInstance | None = None

    def __init__(self):
        self._tables: dict[str, MockNetworkTable] = {}

    @classmethod
    def getDefault(cls) -> "MockNetworkTableInstance":
        """
        Returns the global NT instance.
        """
        if cls.instance is None:
            cls.instance = cls()
        return cls.instance
    
    def getTable(self, key: str) -> MockNetworkTable:
        """
        Gets a table with a specified key.
        """
        if key not in self._tables:
            self._tables[key] = MockNetworkTable(key)
        return self._tables[key]
    
    def getEntry(self, key: str) -> MockNetworkTableEntry:
        """
        Gets the mock entry an a specified key.  
        Uses the default table for entries.
        """
        return self.getTable("").getEntry(key)


@pytest.fixture
def mock_nt_instance() -> MockNetworkTableInstance:
    """
    Provides a mock NetworkTableInstance.
    """
    return MockNetworkTableInstance()


@pytest.fixture
def mock_timer() -> Mock:
    """
    Provides a mock Timer.
    """
    timer = Mock()
    timer.get.return_value = 0.0
    timer.getFPGATimestamp.return_value = 0.0
    return timer


@pytest.fixture
def mock_driver_station() -> Mock:
    """
    Provides a mock DriverStation.
    """
    ds = Mock()
    ds.isEnabled.return_value = True
    ds.isDisabled.return_value = False
    return ds


@pytest.fixture
def mock_wpilib() -> Mock:
    """
    Mocks a wpilib module.
    """
    mock_lib = Mock()
    mock_lib.reportError = Mock()
    mock_lib.Timer = Mock(return_value=Mock(
        get=Mock(return_value=0.0),
        reset=Mock(),
        start=Mock(),
        stop=Mock()
    ))
    mock_lib.DriverStation = Mock()
    mock_lib.DriverStation.isEnabled = Mock(return_value=True)
    mock_lib.DutyCycleEncoder = Mock
    mock_lib.TimedRobot = object
    
    return mock_lib
