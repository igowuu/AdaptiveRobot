from typing import Callable, TypeAlias

from wpilib import SendableChooser, SmartDashboard

from adaptive_robot.autonomous.async_actions import AsyncAction


ActionFactory: TypeAlias = Callable[[], AsyncAction]


class ActionChooser:
    """
    A SendableChooser wrapper that manages the selection of AsyncActions.
    
    Provides a builder-style interface for presenting multiple autonomous
    actions on SmartDashboard and getting the user-selected action at runtime.
    """
    def __init__(self) -> None:
        """
        Creates a new ActionChooser.
        """
        self._chooser = SendableChooser()
        self._options: dict[str, ActionFactory] = {}
        self._default_name: str | None = None
        self._default_factory: ActionFactory | None = None

    def add_option(self, name: str, factory: ActionFactory) -> None:
        """
        Adds an option to the chooser.
        
        :param name: The name of the action (displayed on SmartDashboard).
        :param factory: A callable that creates a new AsyncAction when called.
        :raises ValueError: If the given name already exists within the ActionChooser.
        """
        if name in self._options:
            raise ValueError(f"[ActionChooser] Option '{name}' already exists.")
        
        self._options[name] = factory
        self._chooser.addOption(name, name)

    def set_default(self, name: str, factory: ActionFactory) -> None:
        """
        Sets the default option in the chooser.
        
        :param name: The name of the action.
        :param factory: A callable that creates a new AsyncAction when called.
        """
        if name not in self._options:
            self.add_option(name, factory)
        
        self._default_name = name
        self._default_factory = factory
        self._chooser.setDefaultOption(name, name)

    def get_selected_factory(self) -> ActionFactory:
        """
        Returns the currently selected action factory.
        
        :returns: The factory function for the selected action.
        :raises RuntimeError: If no action selected and no default is set.
        """
        selected_name = self._chooser.getSelected()
        
        if selected_name is None:
            if self._default_factory is None:
                raise RuntimeError(
                    "[ActionChooser] No action selected and no default option set. "
                    "Call set_default() before get_selected_factory()."
                )
            return self._default_factory
        
        if selected_name not in self._options:
            raise ValueError(
                f"[ActionChooser] Selected option '{selected_name}' is not registered. "
                f"Available options: {list(self._options.keys())}"
            )
        
        return self._options[selected_name]

    def get_selected_name(self) -> str:
        """
        Gets the name of the currently selected option.
        
        :returns: The name of the selected option, or the default if none selected.
        :raises RuntimeError: If no action selected and no default is set.
        """
        selected_name = self._chooser.getSelected()
        
        if selected_name is None:
            if self._default_name is None:
                raise RuntimeError(
                    "[ActionChooser] No action selected and no default option set. "
                    "Call set_default() before get_selected_name()."
                )
            return self._default_name
        
        return selected_name

    def get_available_options(self) -> list[str]:
        """
        Returns all available action option names.
        
        :returns: List of all registered option names.
        """
        return list(self._options.keys())

    def publish(self, key: str = "Auto") -> None:
        """
        Publishes the SendableChooser to SmartDashboard.
        
        :param key: The SmartDashboard key to publish under.
        """
        SmartDashboard.putData(key, self._chooser)
