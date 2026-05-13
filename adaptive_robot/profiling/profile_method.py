from functools import wraps
from typing import Callable, Any
from contextvars import ContextVar

from wpilib import Timer

from adaptive_robot.profiling.profiling_models import ProfilingContext, MethodProfile


_profiling_context: ContextVar[ProfilingContext | None] = ContextVar(
    "profiling_context",
    default=None
)


def set_profiling_context(context: ProfilingContext | None) -> None:
    """
    Sets the active profiling context for the current execution context.
    """
    _profiling_context.set(context)


def get_profiling_context() -> ProfilingContext | None:
    """
    Returns the current context for the module.
    """
    return _profiling_context.get()


def profile_method(func: Callable[..., Any]) -> Callable[..., Any]:
    """
    Decorates a method so that its elapsed time is recorded each robot iteration.
    The wrapped method's execution statistics will be stored and written to a file
    periodically or upon the robot entering a disabled state.
    """
    def decorator(function: Callable[..., Any]) -> Callable[..., Any]:
        method_id = function.__qualname__

        @wraps(function)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            context = get_profiling_context()
            start = Timer.getFPGATimestamp()

            try:
                return function(*args, **kwargs)

            finally:
                end = Timer.getFPGATimestamp()
                elapsed = end - start
                if context is not None:
                    if method_id not in context.method_profiles:
                        context.method_profiles[method_id] = MethodProfile(
                            method_name=function.__qualname__,
                            method_id=method_id,
                        )

                    context.method_profiles[method_id].record_execution(elapsed)

        return wrapper

    return decorator(func)
