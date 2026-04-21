from enum import StrEnum
from dataclasses import dataclass


class FaultException(Exception):
    """
    Custom exception to indicate that a Fault object has been raised.
    """
    def __init__(self, fault: Fault) -> None:
        self.fault = fault
        super().__init__(fault.description)


class FaultSeverity(StrEnum):
    """
    Represents the possible fault severities.
    
    WARNING - Logs a fault without impacting the parent AdaptiveComponent or AdaptiveRobot.  
    ERROR - Logs a fault while contributing to the potential of killing a component.
    CRITICAL - Logs a fault that immediately kills the robot.
    """
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


@dataclass
class Fault:
    """
    Represents a single fault object that is logged into an AdaptiveLogger
    and is able to alter scheduler behavior, depending on severity.
    
    consecutive_count is only used during logging to indicate how many
    iterations had this identical fault. Set automatically by FaultLogger.
    """
    component: str | None
    severity: FaultSeverity
    description: str
    timestamp: float
    exception_type: str | None
    traceback: str | None
