# pyright: reportPrivateUsage=false

from pathlib import Path
from unittest.mock import patch

from adaptive_robot.adaptive_robot import AdaptiveRobot
from adaptive_robot.tunable.tunable_value import TunablePersistenceManager, TunableValue


class TestTunablePersistence:
    """
    Tests for tunable persistence across sessions.
    """
    def setup_method(self) -> None:
        TunablePersistenceManager._loaded = False
        TunablePersistenceManager._cache = {}
        TunablePersistenceManager._registry = {}

    def test_persisted_value_is_loaded_when_tunable_is_created(self, tmp_path: Path) -> None:
        """
        Persisted values should override the default when a tunable is recreated.
        """
        TunablePersistenceManager._file_path = tmp_path / "tunables.json"
        TunablePersistenceManager._loaded = False

        persisted_file = tmp_path / "tunables.json"
        persisted_file.write_text('{"persisted/tunable": 1.25}')

        tunable = TunableValue("persisted/tunable", 0.5)

        assert tunable.value == 1.25

    def test_save_all_persists_current_networktable_values(self, tmp_path: Path) -> None:
        """
        Saving all tunables should persist the current NetworkTables values.
        """
        TunablePersistenceManager._file_path = tmp_path / "tunables.json"
        TunablePersistenceManager._loaded = False

        tunable = TunableValue("persist/save", 0.5)
        tunable._entry.setDouble(0.8)

        TunablePersistenceManager.save_all()

        saved_text = (tmp_path / "tunables.json").read_text()
        assert '"persist/save": 0.8' in saved_text

    def test_disabled_exit_triggers_persistence_save(self) -> None:
        """
        The robot should save tunables when disabledExit is called.
        """
        robot = AdaptiveRobot()

        with patch.object(TunablePersistenceManager, "save_all") as mocked_save:
            robot.disabledExit()
            mocked_save.assert_called_once()
