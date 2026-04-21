import traceback
from typing import TYPE_CHECKING, final
from abc import ABC

import wpilib

from adaptive_robot.faults.faults import Fault, FaultException, FaultSeverity

if TYPE_CHECKING:
    from adaptive_robot.adaptive_component.adaptive_component import AdaptiveComponent


class Faultable(ABC):
    """
    Marks a class as able to raise faults, and adds the raise_fault() method
    to easily raise fault objects (rather than creating and raising yourself).
    """
    @final
    def raise_fault(
        self, 
        component: "AdaptiveComponent | None",
        severity: FaultSeverity,
        description: str,
        exception: Exception | None = None
    ) -> None:
        """
        Creates and raises a Fault with standardized format for an optional component.

        :raises: FaultException.
        """
        exception_type = exception.__class__.__name__ if exception is not None else None
        backtrace = traceback.format_exc() if exception is not None else None

        fault_exception = FaultException(
            Fault(
                component=component.__class__.__name__ if component else None,
                severity=severity,
                description=description,
                timestamp=wpilib.Timer.getFPGATimestamp(),
                exception_type=exception_type,
                traceback=backtrace
            )
        )

        if exception is not None:
            raise fault_exception from exception
        else:
            raise fault_exception
