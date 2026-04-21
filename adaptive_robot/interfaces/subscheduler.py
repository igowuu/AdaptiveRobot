from typing import Any
from abc import ABC

from adaptive_robot.interfaces.faultable import Faultable


class Subscheduler(Faultable, ABC):
    """
    Holds the required methods for a subscheduler.
    """
    def __init__(self, *args: Any, **kwargs: Any) -> None: ...
    def run(self, *args: Any, **kwargs: Any) -> None: ...
