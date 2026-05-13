from dataclasses import dataclass, field
import math


@dataclass
class MethodProfile:
    """
    Profiling statistics for a method.
    Tracks call count, timing statistics, and provides mean/standard deviation.
    """
    method_name: str
    method_id: str
    call_count: int = 0
    total_time: float = 0.0
    min_time: float = float('inf')
    max_time: float = 0.0
    sum_of_squares: float = 0.0
    
    @property
    def mean(self) -> float:
        """
        Returns average execution time in seconds.
        """
        return self.total_time / self.call_count if self.call_count > 0 else 0.0
    
    @property
    def stddev(self) -> float:
        """
        Returns standard deviation of execution time in seconds.
        """
        if self.call_count < 2:
            return 0.0
        variance = (self.sum_of_squares / self.call_count) - (self.mean ** 2)
        return math.sqrt(max(0.0, variance))  # max to handle floating point error
    
    def record_execution(self, elapsed_time: float) -> None:
        """
        Records a single method execution.
        """
        self.call_count += 1
        self.total_time += elapsed_time
        self.min_time = min(self.min_time, elapsed_time)
        self.max_time = max(self.max_time, elapsed_time)
        self.sum_of_squares += elapsed_time ** 2
    
    def to_dict(self) -> dict[str, float | str]:
        """
        Converts the profile to a dict.
        """
        return {
            "method_name": self.method_name,
            "method_id": self.method_id,
            "call_count": self.call_count,
            "mean_time_seconds": self.mean,
            "min_time_seconds": self.min_time if self.call_count > 0 else 0.0,
            "max_time_seconds": self.max_time if self.call_count > 0 else 0.0,
            "stddev_seconds": self.stddev
        }


@dataclass
class ProfilingContext:
    """
    Holds necessary attributes for methods decorated in profile_method to communicate 
    with the subscheduler. Context must be injected by the subscheduler to each instance.
    """
    method_profiles: dict[str, MethodProfile] = field(default_factory=dict[str, MethodProfile])
