from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import wpilib

from adaptive_robot.faults.faults import Fault
from adaptive_robot.utils.json_io import log_json_data


fault_loggable_type = int | float | str | bool


class FaultLogger:
    """
    Logs faults to a file each iteration, given a directory.
    Automatically generates session-unique filenames with FRC timestamp format.
    """
    MAX_BUFFER_SIZE = 500  # Max pending faults before warning
    
    def __init__(self, logging_folder: str) -> None:
        log_path = Path(logging_folder)
        try:
            log_path.mkdir(parents=True, exist_ok=True)
        except Exception:
            raise RuntimeError(f"Unable to create fault directory {logging_folder}.")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"FRC_{timestamp}_faults.json"

        self.logging_directory = str(log_path / filename)
        self.faults: list[dict[str, fault_loggable_type]] = []
        self.previous_faults: list[dict[str, fault_loggable_type]] = []
        self._buffer_warning_issued = False

    def run(self, faults: list[Fault]) -> None:
        """
        Logs the given Fault objects into the directory attatched to the scheduler.
        Implements bounded buffering: if pending faults exceed MAX_BUFFER_SIZE, issues warning.

        :param faults: The Fault objects for this robot iteration.
        """
        for fault in faults:
            fault_dict = asdict(fault)
            self.faults.append(fault_dict)

            if len(self.faults) > self.MAX_BUFFER_SIZE and not self._buffer_warning_issued:
                self._buffer_warning_issued = True
                wpilib.reportWarning(
                    f"Fault buffer has exceeded {self.MAX_BUFFER_SIZE} pending faults. "
                    f"Logging may be failing. Current buffer size: {len(self.faults)}"
                )
        if faults:
            try:
                log_json_data(
                    file_path=self.logging_directory, 
                    data=self.faults,
                    indent=2,
                    ensure_ascii=True,
                    append=True
                )
                self.faults = []
                self._buffer_warning_issued = False
            except Exception as e:
                wpilib.reportError(
                    f"Fault logging exception occured: {e}\n"
                    "Trying next iteration..."
                )
