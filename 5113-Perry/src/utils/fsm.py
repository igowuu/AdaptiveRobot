from typing import Callable, Optional, List, Any
from dataclasses import dataclass


@dataclass
class State:
    """
    Represents a single state in a finite state machine.

    :param name: The name of the state.
    :param conditional: A callable returning a bool that determines if this state should be active.
    :param on_update: A callable executed every time the FSM updates while this state is active.
    :param on_enable: Optional callable executed once when this state becomes active.
    :param on_disable: Optional callable executed once when this state is deactivated.
    """
    name: str
    conditional: Callable[..., bool]
    on_update: Callable[..., None]
    on_enable: Optional[Callable[..., None]] = None
    on_disable: Optional[Callable[..., None]] = None


class FiniteStateMachine:
    """
    Simple finite state machine implementation for Perry (FRC 5113 bot).

    The FSM evaluates conditional states in the order they were added and switches
    to the first state whose condition is True. If no conditions are met, it falls back
    to a default state (must be set in your program).
    """

    def __init__(self) -> None:
        self.states: List[State] = []
        self.default_state: Optional[State] = None
        self.current_state: Optional[State] = None

    def set_default_state(
        self,
        name: str,
        on_update: Callable[..., None],
        on_enable: Optional[Callable[..., None]] = None,
        on_disable: Optional[Callable[..., None]] = None,
    ) -> None:
        """
        Set the default state of the FSM.
        The default state is used when no other conditional state is true.

        :param name: Name of the default state.
        :param on_update: Callable executed every update while in the default state.
        :param on_enable: Optional callable executed when the default state is activated.
        :param on_disable: Optional callable executed when the default state is deactivated.
        """
        self.default_state = State(
            name=name,
            conditional=lambda *args, **kwargs: True,
            on_update=on_update,
            on_enable=on_enable,
            on_disable=on_disable,
        )

        # Start FSM in default state
        self.current_state = self.default_state
        if self.current_state.on_enable:
            self.current_state.on_enable()

    def add_state(
        self,
        name: str,
        conditional: Callable[..., bool],
        on_update: Callable[..., None],
        on_enable: Optional[Callable[..., None]] = None,
        on_disable: Optional[Callable[..., None]] = None,
    ) -> None:
        """
        Add a new conditional state to the FSM.

        :param name: Name of the state.
        :param conditional: Callable that returns True if this state should be activated.
        :param on_update: Callable executed each update while in this state.
        :param on_enable: Optional callable executed once when this state becomes active.
        :param on_disable: Optional callable executed once when this state is deactivated.
        """
        self.states.append(
            State(name, conditional, on_update, on_enable, on_disable)
        )

    def _switch_state(self, new_state: State, *args: Any, **kwargs: Any) -> None:
        """
        Switch the current state with a new state.

        :param new_state: State that will replace the current State.
        """
        if self.current_state is not None:
            if self.current_state.on_disable:
                self.current_state.on_disable(*args, **kwargs)

        self.current_state = new_state
        if self.current_state.on_enable:
            self.current_state.on_enable(*args, **kwargs)

    def execute(self, *args: Any, **kwargs: Any) -> Optional[State]:
        """
        Evaluate all conditional states and update the FSM.

        The FSM will switch to the first state whose condition returns True.
        If no conditional states are True, it will fall back to the default state.

        :return: The current active state after evaluation.
        :rtype: Optional[State]
        """
        matched_state = None
        # Find first state whose condition is true
        for state in self.states:
            if state.conditional(*args, **kwargs):
                matched_state = state
                break 
        
        if matched_state is not None:
            if self.current_state is not matched_state:
                self._switch_state(matched_state, *args, **kwargs)
        else:
            if self.current_state is not self.default_state and self.default_state is not None:
                self._switch_state(self.default_state, *args, **kwargs)
        
        if self.current_state:
            self.current_state.on_update(*args, **kwargs)
        return self.current_state
