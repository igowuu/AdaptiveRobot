from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING

import wpilib

from adaptive_robot.utils.json_io import log_json_data

if TYPE_CHECKING:
    from adaptive_robot.profiling.profiling_models import MethodProfile


class ProfilingLogger:
    """
    Logs collected method profiling data to a JSON file periodically and on disable.
    Automatically generates unique filenames with FRC timestamp format.
    """
    MAX_BUFFER_SIZE = 100
    
    def __init__(self, logging_folder: str) -> None:
        log_path = Path(logging_folder)
        try:
            log_path.mkdir(parents=True, exist_ok=True)
        except Exception:
            raise RuntimeError(f"Unable to create profiling directory {logging_folder}.")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"FRC_{timestamp}_profiles.json"

        self.logging_directory = str(log_path / filename)
        self.pending_snapshots: list[dict[str, str | list[dict[str, str | float]]]] = []
        self._buffer_warning_issued = False

    def log_profiles(self, method_profiles: dict[str, "MethodProfile"], append: bool = False) -> None:
        """
        Logs a snapshot of current method profiles. 
        Replaces file by default (append=False) for real-time updates.
        Use append=True on disable to preserve final performance snapshot.
        
        :param method_profiles: Dictionary of MethodProfile objects keyed by method_id.
        :param append: If True, append to file. If False, replace file content.
        """
        if not method_profiles:
            return
        
        snapshot: dict[str, str | list[dict[str, str | float]]] = {
            "timestamp": datetime.now().isoformat(),
            "methods": [profile.to_dict() for profile in method_profiles.values()]
        }
        
        self.pending_snapshots.append(snapshot)
        
        if len(self.pending_snapshots) > self.MAX_BUFFER_SIZE and not self._buffer_warning_issued:
            self._buffer_warning_issued = True
            wpilib.reportWarning(
                f"Profiling buffer has exceeded {self.MAX_BUFFER_SIZE} pending snapshots. "
                f"Logging might be failing. Current buffer size: {len(self.pending_snapshots)}"
            )
        
        try:
            log_json_data(
                file_path=self.logging_directory,
                data=self.pending_snapshots,
                indent=2,
                ensure_ascii=True,
                append=append
            )
            self.pending_snapshots = []
            self._buffer_warning_issued = False
        except Exception as e:
            wpilib.reportError(
                f"Profiling logging exception occurred: {e}\n"
                "Trying next iteration..."
            )
