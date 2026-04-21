from adaptive_robot.interfaces.subscheduler import Subscheduler
from adaptive_robot.faults.faults import FaultSeverity, FaultException
from adaptive_robot.autonomous.instructions import Instruction

from adaptive_robot.autonomous.async_actions import AsyncAction


class ActionScheduler(Subscheduler):
    """
    Executes and manages the lifecycle of scheduled actions.
    """
    def __init__(self) -> None:
        self._actions: dict[str, AsyncAction] = {}
        self._active_instructions: dict[str, Instruction] = {}

    def schedule(self, action: AsyncAction, name: str) -> str | None:
        """
        Adds an action to be executed by the scheduler.

        :returns: The name of the action, None if an action with the name is already running.
        """
        if name in self._actions:
            return None

        self._actions[name] = action
        return name
    
    def run(self) -> None:
        """
        Runs one scheduler iteration over all scheduled actions.

        :raises Exception: Upon any error, forcefully cancelling the action cleanly.
        """
        for name, action in list(self._actions.items()):
            try:
                instruction = action.send(None)
                
                if instruction:
                    self._step_instruction(name, instruction)
                    
                    if instruction.is_complete():
                        instruction.on_complete()
                        self._active_instructions.pop(name, None)
                    else:
                        self._active_instructions[name] = instruction
                        
            except StopIteration:
                self._active_instructions.pop(name, None)
                self._actions.pop(name, None)
            except FaultException:
                raise
            except Exception as e:
                if name in self._actions:
                    self._cleanup_action(name)
                
                self.raise_fault(
                    component=None, 
                    severity=FaultSeverity.CRITICAL, 
                    description=(
                        f"Unknown fault in ActionScheduler raised: {e}\n"
                        "Action was successfully cancelled and removed."
                    ),
                    exception=e
                )

    def _step_instruction(self, name: str, instruction: Instruction) -> None:
        """
        Executes one iteration of an instruction.
        """
        try:
            instruction.step()
        except Exception as e:
            self._cleanup_action(name)
            self.raise_fault(
                component=None,
                severity=FaultSeverity.CRITICAL,
                description=(
                    f"Instruction.step() failed for action {name}: {e}\n"
                    "Action was successfully cancelled and removed."
                ),
                exception=e
            )

    def _cleanup_action(self, name: str) -> None:
        """
        Cleans up an action and its associated instruction.
        """
        if name in self._active_instructions:
            instr = self._active_instructions[name]
            try:
                instr.cleanup()
            except Exception:
                pass
            self._active_instructions.pop(name, None)

        if name in self._actions:
            try:
                self._actions[name].close()
            except (GeneratorExit, StopIteration):
                pass
            self._actions.pop(name, None)

    def cancel(self, name: str) -> str | None:
        """
        Cancels a specific scheduled action.

        :returns: The name of the action, None if it does not exist.
        """
        if name not in self._actions:
            return None

        try:
            self._cleanup_action(name)
        except FaultException:
            raise
        except Exception as e:
            self.raise_fault(
                component=None, 
                severity=FaultSeverity.CRITICAL, 
                description=(
                    f"Unknown fault in ActionScheduler raised: {e}"
                    f" when trying to cancel action {name}!"
                ),
                exception=e
            )
        
        return name

    def cancel_all(self) -> list[str] | None:
        """
        Cancels and clears all scheduled actions.

        :returns: A list of the cancelled names, or None if none were cancelled.
        """
        if not self._actions:
            return None

        all_names: list[str] = []

        for name in list(self._actions):
            try:
                all_names.append(name)
                self._cleanup_action(name)
            except FaultException:
                raise
            except Exception as e:
                self.raise_fault(
                    component=None, 
                    severity=FaultSeverity.CRITICAL, 
                    description=(
                        f"Unknown fault in ActionScheduler raised: {e}"
                        " when trying to cancel all actions!"
                    ),
                    exception=e
                )

        return all_names

    def is_running(self, name: str) -> bool:
        """
        Check if an action is still running.
        """
        return name in self._actions

    def get_running_actions(self) -> list[AsyncAction]:
        """
        Returns all of the currently running actions.
        """
        return list(self._actions.values())
