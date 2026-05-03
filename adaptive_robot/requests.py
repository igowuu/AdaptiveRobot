from dataclasses import dataclass, field
import re
from enum import Enum

from wpilib import Timer
from wpimath.units import seconds


@dataclass(frozen=True)
class Request:
    """
    Stores internal and accessible values for each individual request.
    """
    value: float
    priority: int
    timeout: seconds = 0.2
    source: str = "unknown"
    timestamp: seconds = field(default_factory=Timer.getFPGATimestamp)


class BasicPriority(Enum):
    """
    Represents a basic priority where SAFETY overrides everything and AUTO overrides TELEOP.
    """
    SAFETY = 3
    AUTO = 2
    TELEOP = 1


class RequestArbitrator:
    """
    Manages a single entity via request-based arbitration.
    """
    def __init__(
        self,
        default_value: float = 0.0,
        default_priority: int = -1,
        default_source: str = 'default'
    ) -> None:
        """
        Creates an RequestArbitrator object, used to manage, add, and configure requests each iteration
        in robot components.
        
        :param default_value: The default value that the RequestArbitrator will reset to if no persisting
        requests are made.
        :param default_priority: The default priority that the RequestArbitrator will reset to if no 
        persisting requests are made.
        :param default_source: The default source that the RequestArbitrator will reset to if no 
        persisting requests are made.
        """
        self._requests: dict[str, Request] = {}
        self._default = Request(default_value, default_priority, float('inf'), default_source)

        self._enabled = True
        self._prev_request = self._default

    def _validate_source(self, source: str) -> None:
        """
        Validates the source name format.

        :raises ValueError: If the source name is invalid.
        """
        if not source:
            raise ValueError("Source name cannot be empty.")
        
        if len(source) > 50:
            raise ValueError(f"Source name exceeds maximum length of 50 characters: '{source}'")
        
        if not re.match(r'^[a-zA-Z0-9_]+$', source):
            raise ValueError(
                f"Source name '{source}' contains invalid characters. "
                f"Only letters, numbers, and underscores are allowed."
            )
    
    def _validate_priority(self, priority: int) -> None:
        """
        Validates that a given priority is not equivalent to the default priority.
        
        :raises RuntimeError: If the default priority and given priority are equivelent.
        """
        if self._default.priority == priority:
            raise RuntimeError(
                f"Priority of {priority} is equivalent to default priority of "
                f"{self._default.priority}."
            )

    def request(
        self, 
        value: float,
        priority: int,
        source: str = "unknown",
        timeout: seconds = 0.2
    ) -> None:
        """
        Submits a request to the RequestArbitrator.

        :param value: The assigned value to the request.
        :param priority: The priority that the request has against other requests in the RequestArbitrator.
        The priority should ideally be positive and must be different than the default priority.
        :param source: The name assigned to the request. Must be alphanumeric with underscores, max 50 chars.
        :param timeout: The duration (in seconds) that this request remains active. Must be larger than zero
        and smaller than or equal to 10.0 seconds.

        :raises ValueError: If source name is invalid or timeout is out of bounds.
        :raises RuntimeError: If the given priority is the same as the default priority.
        """
        if timeout <= 0 or timeout > 10.0:
            raise ValueError(
                f"Timeout must be larger than 0s and smaller than 10.0s, got {timeout}s. "
                f"This parameter controls how long a request remains active in the RequestArbitrator."
            )
        
        if not self._enabled:
            self.clear()
            return

        self._validate_source(source)
        self._validate_priority(priority)

        request = Request(
            value=value,
            priority=priority,
            source=source,
            timeout=timeout,
        )

        self._requests[request.source] = request

    def clear(self) -> None:
        """
        Clears all pending requests in the RequestArbitrator.
        """
        self._requests.clear()

    def resolve(self) -> Request:
        """
        Returns the request with the highest priority within the RequestArbitrator object.  
        If requests have the same highest priority, returns the most recent request (by timestamp).
        If requests are made in the same iteration (same timestamp), returns the most recently 
        inserted request (tie-breaker).  
        If the RequestArbitrator is disabled, always returns the default request.
        """
        if not self._enabled:
            self._prev_request = self._default
            return self._default

        now = Timer.getFPGATimestamp()

        valid_requests: list[Request] = []

        for source, request in list(self._requests.items()):
            if now - request.timestamp > request.timeout:
                del self._requests[source]
            else:
                valid_requests.append(request)

        if not valid_requests:
            self._prev_request = self._default
            return self._default

        # If tie, most recent request wins.
        winning_request = max(
            valid_requests,
            key=lambda r: (r.priority, r.timestamp)
        )

        self._prev_request = winning_request
        return winning_request

    def set_enabled(self, enabled: bool) -> None:
        """
        Sets the enabled state of the RequestArbitrator based on the given bool.
        """
        self._enabled = enabled

    def is_enabled(self) -> bool:
        """
        Returns the enabled state of the RequestArbitrator.
        """
        return self._enabled

    def get_default_request(self) -> Request:
        """
        Returns the default request used when no valid requests are present.
        """
        return self._default

    def get_pending_request_count(self) -> int:
        """
        Returns the number of currently pending requests in the arbitrator.
        """
        return len(self._requests)

    @property
    def last_request(self) -> Request:
        """
        Returns the most recent resolved request of the RequestArbitrator.
        """
        return self._prev_request
