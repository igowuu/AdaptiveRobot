# pyright: reportPrivateUsage=false

from unittest.mock import patch
from adaptive_robot.tunable.tunable_value import TunableValue
from adaptive_robot.tunable.tunable_pid_controller import TunablePIDController


class TestTunableValueFloat:
    """
    Tests for TunableValue with float type.
    """
    def test_float_creation_and_access(self) -> None:
        """
        Tests creating and accessing a float TunableValue.
        """
        tunable = TunableValue("test/float", 0.5)
        assert tunable.value == 0.5
        assert tunable._default == 0.5
    
    def test_float_update_no_change(self) -> None:
        """
        Tests update when value hasn't changed.
        """
        tunable = TunableValue("test/float", 0.5)
        changed = tunable.update()
        assert changed is False
    
    def test_float_update_with_change(self) -> None:
        """
        Tests update when value has changed.
        """
        tunable = TunableValue("test/float", 0.5)
        tunable._entry.setDouble(0.8)
        
        changed = tunable.update()
        assert changed is True
        assert tunable.value == 0.8
    
    def test_float_multiple_updates(self) -> None:
        """
        Tests multiple updates.
        """
        tunable = TunableValue("test/float", 0.5)
        
        tunable._entry.setDouble(0.6)
        assert tunable.update() is True
        assert tunable.value == 0.6
        
        # No change
        assert tunable.update() is False
        
        # Another change
        tunable._entry.setDouble(0.9)
        assert tunable.update() is True
        assert tunable.value == 0.9


class TestTunableValueInt:
    """
    Tests for TunableValue with int type.
    """
    def test_int_creation(self) -> None:
        """
        Tests creating an integer TunableValue.
        """
        tunable = TunableValue("test/int", 42)
        assert tunable.value == 42
        assert tunable._default == 42
    
    def test_int_update(self) -> None:
        """
        Tests integer value updates.
        """
        tunable = TunableValue("test/int", 42)
        tunable._entry.setInteger(100)
        
        changed = tunable.update()
        assert changed is True
        assert tunable.value == 100


class TestTunableValueBool:
    """
    Tests for TunableValue with bool type.
    """
    def test_bool_creation_true(self) -> None:
        """
        Tests creating a boolean TunableValue set to true.
        """
        tunable = TunableValue("test/bool", True)
        assert tunable.value is True
    
    def test_bool_creation_false(self) -> None:
        """
        Tests creating a boolean TunableValue set to false.
        """
        tunable = TunableValue("test/bool", False)
        assert tunable.value is False
    
    def test_bool_update(self) -> None:
        """
        Tests boolean value updates.
        """
        tunable = TunableValue("test/bool", True)
        tunable._entry.setBoolean(False)
        
        changed = tunable.update()
        assert changed is True
        assert tunable.value is False


class TestTunableValueString:
    """
    Tests for TunableValue with string type.
    """
    def test_string_creation(self) -> None:
        """
        Tests creating a string TunableValue.
        """
        tunable = TunableValue("test/string", "default")
        assert tunable.value == "default"
    
    def test_string_update(self) -> None:
        """
        Tests string value updates.
        """
        tunable = TunableValue("test/string", "default")
        tunable._entry.setString("updated")
        
        changed = tunable.update()
        assert changed is True
        assert tunable.value == "updated"


class TestTunableValueNetworkTables:
    """Tests for NetworkTables integration."""
    
    def test_different_keys_separate_entries(self) -> None:
        """
        Tests that different keys maintain separate entries.
        """
        tunable1 = TunableValue("test/value1", 0.5)
        tunable2 = TunableValue("test/value2", 0.8)
        
        # Update the NetworkTable entry
        tunable1._entry.setDouble(1.0)
        # Call update() to refresh the cached value from NetworkTable
        updated = tunable1.update()
        
        assert updated is True  # Value changed
        assert tunable1.value == 1.0
        assert tunable2.value == 0.8  # Should be unchanged


class TestTunablePIDControllerBasic:
    """
    Tests for TunablePIDController basic functionality.
    """
    def test_creation(self) -> None:
        """
        Tests creating a TunablePIDController.
        """
        kp = TunableValue("pid/kp", 1.0)
        ki = TunableValue("pid/ki", 0.0)
        kd = TunableValue("pid/kd", 0.1)
        
        controller = TunablePIDController(kp, ki, kd)
        
        assert controller._kp == kp
        assert controller._ki == ki
        assert controller._kd == kd
    
    def test_string_representation(self) -> None:
        """
        Tests string representation.
        """
        kp = TunableValue("pid/kp", 1.0)
        ki = TunableValue("pid/ki", 0.5)
        kd = TunableValue("pid/kd", 0.1)
        
        controller = TunablePIDController(kp, ki, kd)
        str_rep = str(controller)
        
        assert "1.0" in str_rep
        assert "0.5" in str_rep
        assert "0.1" in str_rep


class TestTunablePIDControllerUpdate:
    """
    Tests for TunablePIDController updates.
    """
    def test_no_change_no_update(self) -> None:
        """
        Tests that setPID is not called when no values changed.
        """
        kp = TunableValue("pid/kp", 1.0)
        ki = TunableValue("pid/ki", 0.0)
        kd = TunableValue("pid/kd", 0.1)
        
        controller = TunablePIDController(kp, ki, kd)
        
        with patch.object(controller, 'setPID') as mock_set_pid:
            controller.update_from_tunables()
            mock_set_pid.assert_not_called()
    
    def test_kp_change_triggers_update(self) -> None:
        """
        Tests that Kp change triggers setPID.
        """
        kp = TunableValue("pid/kp", 1.0)
        ki = TunableValue("pid/ki", 0.0)
        kd = TunableValue("pid/kd", 0.1)
        
        controller = TunablePIDController(kp, ki, kd)
        
        kp._entry.setDouble(2.0)
        
        with patch.object(controller, 'setPID') as mock_set_pid:
            controller.update_from_tunables()
            mock_set_pid.assert_called_once()
            # Check that new values were passed
            call_args = mock_set_pid.call_args[0]
            assert call_args[0] == 2.0  # kp
    
    def test_ki_change_triggers_update(self) -> None:
        """
        Tests that Ki change triggers setPID.
        """
        kp = TunableValue("pid/kp", 1.0)
        ki = TunableValue("pid/ki", 0.0)
        kd = TunableValue("pid/kd", 0.1)
        
        controller = TunablePIDController(kp, ki, kd)
        
        ki._entry.setDouble(0.5)
        
        with patch.object(controller, 'setPID') as mock_set_pid:
            controller.update_from_tunables()
            mock_set_pid.assert_called_once()
            call_args = mock_set_pid.call_args[0]
            assert call_args[1] == 0.5  # ki
    
    def test_kd_change_triggers_update(self) -> None:
        """
        Tests that Kd change triggers setPID.
        """
        kp = TunableValue("pid/kp", 1.0)
        ki = TunableValue("pid/ki", 0.0)
        kd = TunableValue("pid/kd", 0.1)
        
        controller = TunablePIDController(kp, ki, kd)
        
        kd._entry.setDouble(0.2)
        
        with patch.object(controller, 'setPID') as mock_set_pid:
            controller.update_from_tunables()
            mock_set_pid.assert_called_once()
            call_args = mock_set_pid.call_args[0]
            assert call_args[2] == 0.2  # kd
    
    def test_multiple_changes_single_update(self) -> None:
        """
        Tests that multiple coefficient changes are detected.
        """
        kp = TunableValue("pid/kp", 1.0)
        ki = TunableValue("pid/ki", 0.0)
        kd = TunableValue("pid/kd", 0.1)
        
        controller = TunablePIDController(kp, ki, kd)
        
        kp._entry.setDouble(2.0)
        ki._entry.setDouble(0.5)
        kd._entry.setDouble(0.2)
        
        with patch.object(controller, 'setPID') as mock_set_pid:
            controller.update_from_tunables()
            mock_set_pid.assert_called_once()
            call_args = mock_set_pid.call_args[0]
            assert call_args == (2.0, 0.5, 0.2)
    
    def test_persistent_update_tracking(self) -> None:
        """
        Tests that each tunable tracks its own updates.
        """
        kp = TunableValue("pid/kp", 1.0)
        ki = TunableValue("pid/ki", 0.0)
        kd = TunableValue("pid/kd", 0.1)
        
        controller = TunablePIDController(kp, ki, kd)
        
        # First change to Kp
        kp._entry.setDouble(2.0)
        controller.update_from_tunables()
        
        # Second call should not detect change
        with patch.object(controller, 'setPID') as mock_set_pid:
            controller.update_from_tunables()
            mock_set_pid.assert_not_called()
        
        # Change Kd
        kd._entry.setDouble(0.2)
        with patch.object(controller, 'setPID') as mock_set_pid:
            controller.update_from_tunables()
            mock_set_pid.assert_called_once()
