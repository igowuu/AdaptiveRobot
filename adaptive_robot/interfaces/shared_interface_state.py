from typing import final


class SharedHealthState:
    """
    Shared base for all capability interfaces.
    """
    def __init_subclass__(cls) -> None:
        """
        Auto-initializes health state when a subclass is created.
        """
        super().__init_subclass__()
        cls._is_healthy = True

    @final
    def set_health(self, is_healthy: bool) -> None:
        """
        Used by the component subscheduler to make the object healthy or unhealthy.  
        The user should not call this method.
        """
        self._is_healthy = is_healthy

    @final
    def is_healthy(self) -> bool:
        """
        Returns True if the object is healthy.
        """
        return self._is_healthy
