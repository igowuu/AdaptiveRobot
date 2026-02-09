from dataclasses import dataclass, field

from wpilib import Timer
from wpimath.units import seconds


@dataclass(frozen=True)
class AxisRequest:
    """
    Stores internal and accessible values for each individual request, made by the user.
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
    def __init__(
        self,
        default_value: float = 0.0,
        default_priority: int = -1,
        default_source: str = 'default'
    ) -> None:
        """
        Creates an AxisController object, used to manage, add, and configure requests each iteration
        in robot components.
        
        :param default_value: The default value that the AxisController will reset to if no persisting
        requests are made.

        :param default_priority: The default priority that the AxisController will reset to if no 
        persistingrequests are made.
        
        :param default_source: The default source that the AxisController will reset to if no 
        persistingrequests are made.
        """
        self._requests: dict[str, AxisRequest] = {}
        self._default = AxisRequest(default_value, default_priority, float('inf'), default_source)

        self._enabled = True
        self._last_request = self._default

    def request(
        self, 
        value: float,
        priority: int,
        source: str = "unknown"
    ) -> None:
        """
        Submits a request to the AxisController.

        :param value: The assigned value to the request.
        :param priority: The priority that the request has against other requests in the AxisController.
        :param source: The name assigned to the request.
        """
        if not self._enabled:
            self.clear()
            return

        request = AxisRequest(
            value=value,
            priority=priority,
            source=source
        )

        self._requests[request.source] = request

    def clear(self) -> None:
        """
        Clears all pending requests in the AxisController.
        """
        self._requests.clear()

    def resolve(self) -> AxisRequest:
        """
        Returns the request with the highest priority within the AxisController object.
        """
        if not self._enabled:
            self._last_request = self._default
            return self._default

        now = Timer.getFPGATimestamp()

        # Add requests within designated time period to valid_requests
        valid_requests: list[AxisRequest] = []

        for request in self._requests.values():
            if now - request.timestamp <= request.timeout:
                valid_requests.append(request)

        if not valid_requests:
            self._last_request = self._default
            return self._default

        winning_request = max(
            valid_requests,
            key=lambda r: (r.priority, r.timestamp)
        )

        self._last_request = winning_request
        return winning_request

    def set_enabled(self, enabled: bool) -> None:
        """
        Sets the enabled state of the AxisController based on the given bool.
        """
        self._enabled = enabled

    @property
    def last_request(self) -> AxisRequest:
        """
        Returns the most recent resolved request of the AxisController.
        """
        return self._last_request
