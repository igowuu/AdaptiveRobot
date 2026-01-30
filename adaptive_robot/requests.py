from dataclasses import dataclass, field

from wpilib import Timer
from wpimath.units import seconds


@dataclass(frozen=True)
class AxisRequest:
    """
    Creates an AxisRequest object.
    """
    value: float
    priority: int
    timeout: seconds = 0.2
    source: str = "unknown"
    timestamp: seconds = field(default_factory=Timer.getFPGATimestamp)


class AxisController:
    """
    Manages a single logical axis using request-based arbitration.
    One active request allowed per axis.
    """
    def __init__(self) -> None:
        self._requests: dict[str, AxisRequest] = {}
        self._default = AxisRequest(0.0, -1, float('inf'), 'default')

        self._enabled = True
        self._last_output = self._default.value

    def request(self, request: AxisRequest) -> None:
        """Submit or replace a request for this source."""
        if not self._enabled:
            return

        self._requests[request.source] = request

    def clear(self) -> None:
        """Clears all pending requests."""
        self._requests.clear()

    def resolve(self) -> AxisRequest:
        """
        Resolves requests and returns the winning AxisRequest object.
        """
        if not self._enabled:
            self._last_output = self._default.value
            return self._default

        now = Timer.getFPGATimestamp()

        # Add requests within designated time period to valid_requests
        valid_requests: list[AxisRequest] = []

        for request in self._requests.values():
            if now - request.timestamp <= request.timeout:
                valid_requests.append(request)

        if not valid_requests:
            self._last_output = self._default.value
            return self._default

        winning_request = max(
            valid_requests,
            key=lambda r: (r.priority, r.timestamp)
        )

        self._last_output = winning_request.value
        return winning_request

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled

    def reset(self) -> None:
        self._last_output = self._default.value

    @property
    def last_output(self) -> float:
        return self._last_output
