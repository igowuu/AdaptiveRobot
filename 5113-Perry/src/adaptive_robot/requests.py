from typing import Optional

from wpilib import Timer
from wpimath.units import seconds
from wpimath.filter import SlewRateLimiter

class AxisRequest:
    def __init__(
        self,
        value: float ,
        priority: int,
        timeout: seconds = 0.2,
        source: str = "unknown"
    ) -> None:
        """
        Represents a request to move an axis.
        
        :param value: the requested velocity/position for this axis
        :param priority: higher numbers take precedence
        :param timeout: the time in seconds that it takes for a request to expire if not refreshed
        :param source: optional string for telemetry
        """
        self.value = value
        self.priority = priority
        self.timeout = timeout
        self.source = source

        self.timestamp = Timer.getFPGATimestamp()

def resolve_axis(requests: list[AxisRequest]) -> float:
    """
    Returns the requested value with the highest priority of an axis. 
    If axis priorities are the same, the latest request is resolved.
    """
    now = Timer.getFPGATimestamp()
    valid_requests = [
        r for r in requests if now - r.timestamp <= r.timeout
    ]

    if not valid_requests:
        return 0.0

    return max(
        valid_requests,
        key=lambda r: (r.priority, r.timestamp)
    ).value


class AxisController:
    """
    Manages a single logical axis using request-based arbitration.
    """
    def __init__(
        self,
        default: float = 0.0,
        slew_rate: Optional[float] = None
    ) -> None:
        self._requests: list[AxisRequest] = []
        self._default = default

        self._slew: Optional[SlewRateLimiter] = (
            SlewRateLimiter(slew_rate) if slew_rate is not None else None
        )

        self._enabled = True
        self._last_output = default

    def request(self, request: AxisRequest) -> None:
        """Submit a new request to this axis."""
        if self._enabled:
            self._requests.append(request)

    def clear(self) -> None:
        """Clears all pending requests."""
        self._requests.clear()

    def _resolve(self) -> float:
        """
        Resolves requests, applies slew if enabled, and returns axis object.
        """
        if not self._enabled:
            self._last_output = self._default
            return self._default

        value = resolve_axis(self._requests)

        if self._slew is not None:
            value = self._slew.calculate(value)

        self._last_output = value
        return value

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled

    def set_slew_rate(self, rate: Optional[float]) -> None:
        """
        Enable or disable slew.
        Pass None to disable slew.
        """
        self._slew = SlewRateLimiter(rate) if rate is not None else None

    def reset(self) -> None:
        """Resets slew state and output history."""
        if self._slew is not None:
            self._slew.reset(self._default)
        self._last_output = self._default

    @property
    def last_output(self) -> float:
        return self._last_output
