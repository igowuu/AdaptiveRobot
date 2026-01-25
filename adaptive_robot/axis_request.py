from dataclasses import dataclass

@dataclass
class AxisRequest:
    """
    Represents a request to move an axis.
    
    :param value: the requested velocity/position for this axis
    :param priority: higher numbers take precedence
    :param source: optional string for telemetry
    """
    value: float
    priority: int
    source: str = "unknown"

def resolve_axis(requests: list[AxisRequest]) -> float:
    """
    Returns the requested value with the highest priority of an axis. 
    If axis priorities are the same, tie breaking is undefined.
    """
    if not requests:
        return 0.0

    return max(requests, key=lambda r: r.priority).value
