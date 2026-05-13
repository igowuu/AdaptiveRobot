# AdaptiveRobot - FRC architecture
# Copyright (c) 2026 Jacob Taylor (igowu) <https://github.com/igowuu>
# Code from: <https://github.com/igowuu/AdaptiveRobot.git>

# Licensed under the MIT License.
# See https://opensource.org/licenses/MIT for details.

from typing import final, TypeAlias
from dataclasses import dataclass

import wpilib

from wpimath.units import seconds

from adaptive_robot.telemetry.telemetry import TelemetryPublisher
from adaptive_robot.telemetry.struct_telemetry import TelemetryStructPublisher
from adaptive_robot.adaptive_component.adaptive_component import AdaptiveComponent
from adaptive_robot.autonomous.async_actions import AsyncAction
from adaptive_robot.autonomous.action_scheduler import ActionScheduler
from adaptive_robot.faults.fault_scheduler import FaultLogger
from adaptive_robot.fault_manager import FaultManager
from adaptive_robot.profiling.profiling_subscheduler import ProfilingSubscheduler

from adaptive_robot.interfaces.telemetry_interface.telemetry_publishable import TelemetryPublishable
from adaptive_robot.interfaces.telemetry_interface.telemetry_subscheduler import TelemetrySubscheduler
from adaptive_robot.interfaces.tunable_interface.tunable_publishable import TunablePublishable
from adaptive_robot.interfaces.tunable_interface.tunable_subscheduler import TunableSubscheduler
from adaptive_robot.tunable.tunable_value import TunablePersistenceManager
from adaptive_robot.interfaces.schedulable_interface.schedulable import Schedulable
from adaptive_robot.interfaces.schedulable_interface.schedulable_subscheduler import SchedulableSubscheduler


SUPPORTED_INTERFACE: TypeAlias = TelemetryPublishable | TunablePublishable | Schedulable


class RobotDisable(Exception):
    """
    Custom exception to indicate that the robot was intentionally disabled.
    """
    def __init__(self, description: str) -> None:
        super().__init__(description)


@dataclass
class RobotConfig:
    """
    Holds all configuration parameters for an AdaptiveRobot.
    The user can alter this if desired.

    :param period: The amount of seconds each robot iteration takes.  
    :param fault_logging_folder: The folder in the directory in which the program is
    run that fault files will be stored under.  
    :param default_fault_threshold: The default amount of consecutive faults before a
    Schedulable will become unhealthy. This can be changed per subclass via the `fault_threshold` property.  
    :param profile_logging_folder: The folder in the directory in which the program is
    run that profiling files will be stored under.  
    :param profiling_enabled: Determines if anything in the program can be profiled.  
    :param profile_logging_frequency: How often profiling will be automatically logged to a file.  
    :param telemetry_rounding_digits: The amount of decimal digits that all telemetry published
    in TelemetryPublishable interfaces will round to.  
    :param auto_discover_interfaces: Determines if interfaces get auto-discovered recursively at 
    initialization or not. Disabling this will require manual interface registration.  
    :param verbose_discovery: Prints all discovered interfaces to the terminal.  
    """
    period: seconds = 0.02

    fault_logging_folder: str = "faults/"
    default_fault_threshold: int = 10

    profile_logging_folder: str = "profiling/"
    profiling_enabled: bool = True
    profile_logging_frequency: seconds = 10

    telemetry_rounding_digits: int = 5

    auto_discover_interfaces: bool = True
    verbose_discovery: bool = False


class AdaptiveRobot(wpilib.TimedRobot):
    """
    Wrapper for TimedRobot that allows for a dynamic structure while enforcing safety measures 
    and quality-of-life improvements.  

    Users should inherit from this to declare a robot and must call the parent constructor.
    """
    def __init__(self, config: RobotConfig = RobotConfig()) -> None:
        """
        Creates an AdaptiveRobot, which schedules components each iteration and
        enables the execution of all other Adaptive features.

        :param config: The RobotConfig object, which can be customly created
        by the user if desired.
        """
        super().__init__(config.period)

        self._config = config

        self._telemetry_publisher = TelemetryPublisher(config.telemetry_rounding_digits)
        self._telemetry_struct_publisher = TelemetryStructPublisher()

        self._action_scheduler = ActionScheduler()

        self._fault_logger = FaultLogger(config.fault_logging_folder)

        self._telemetry_publishables: list[TelemetryPublishable] = []
        self._tunable_publishables: list[TunablePublishable] = []
        self._schedulables: list[Schedulable] = []

        self._telemetry_subscheduler: TelemetrySubscheduler | None = None
        self._tunable_subscheduler: TunableSubscheduler | None = None
        self._schedulable_subscheduler: SchedulableSubscheduler | None = None

        self._fault_manager: FaultManager | None = None

        self._profiling_subscheduler = ProfilingSubscheduler(
            logging_folder=config.profile_logging_folder,
            logging_enabled=config.profiling_enabled,
            logging_frequency=config.profile_logging_frequency,
            period=config.period
        )

        self._scheduler_initialized = False

    def _discover_interface_implementers(self, interface_type: type[SUPPORTED_INTERFACE]) -> list[SUPPORTED_INTERFACE]:
        """
        Recursively discovers all objects in the robot that implement the given interface.

        :param interface_type: The interface class to search for (TelemetryPublishable, TunablePublishable, or Schedulable).
        :returns: List of all discovered implementers.
        """
        implementers: list[SUPPORTED_INTERFACE] = []
        visited: set[int] = set()
        
        def traverse(obj: object) -> None:
            if id(obj) in visited:
                return

            visited.add(id(obj))

            if isinstance(obj, interface_type):
                implementers.append(obj)

            if hasattr(obj, '__dict__'):
                try:
                    for attr_value in obj.__dict__.values():
                        if attr_value is not None and not isinstance(attr_value, type):
                            traverse(attr_value)
                except (AttributeError, RuntimeError):
                    pass

        traverse(self)
        return implementers
    
    def _auto_discover_all_interfaces(self) -> None:
        """
        Auto-discovers all objects implementing TelemetryPublishable, TunablePublishable, and Schedulable.
        Objects can also be manually registered before this is called.
        Called after onRobotInit() but before subscheduler initialization.
        """
        # Discover and add to existing lists
        for obj in self._discover_interface_implementers(TelemetryPublishable):
            if isinstance(obj, TelemetryPublishable) and obj not in self._telemetry_publishables:
                self._telemetry_publishables.append(obj)
        
        for obj in self._discover_interface_implementers(TunablePublishable):
            if isinstance(obj, TunablePublishable) and obj not in self._tunable_publishables:
                self._tunable_publishables.append(obj)
        
        for obj in self._discover_interface_implementers(Schedulable):
            if isinstance(obj, Schedulable) and obj not in self._schedulables:
                self._schedulables.append(obj)
        
        if self._config.verbose_discovery:
            telemetry_names = [obj.__class__.__name__ for obj in self._telemetry_publishables]
            tunable_names = [obj.__class__.__name__ for obj in self._tunable_publishables]
            schedulable_names = [obj.__class__.__name__ for obj in self._schedulables]
            
            report = (
                "\n[AdaptiveRobot Discovery]\n"
                f"  TelemetryPublishable: {len(self._telemetry_publishables)} {telemetry_names}\n"
                f"  TunablePublishable: {len(self._tunable_publishables)} {tunable_names}\n"
                f"  Schedulable: {len(self._schedulables)} {schedulable_names}\n"
            )
            wpilib.reportWarning(report)

            if len(self._schedulables) == 0:
                wpilib.reportWarning("[AdaptiveRobot] No Schedulables discovered.")

    @final
    def register_telemetry_publishable(self, obj: TelemetryPublishable) -> None:
        """
        Manually registers a TelemetryPublishable object to have its telemetry published.
        Must be registered before robotPeriodic() is first called.
        
        :param obj: The TelemetryPublishable to register.
        :raises RuntimeError: If called after scheduler initialization.
        """
        if self._scheduler_initialized:
            raise RuntimeError(
                "Cannot register objects after scheduler initialization. "
                "Register objects in robotInit() before the scheduler starts."
            )
        if obj not in self._telemetry_publishables:
            self._telemetry_publishables.append(obj)

    @final
    def unregister_telemetry_publishable(self, obj: TelemetryPublishable) -> None:
        """
        Unregisters a TelemetryPublishable object. Can only be called before robotPeriodic().
        
        :param obj: The TelemetryPublishable to unregister.
        :raises RuntimeError: If called after scheduler initialization.
        """
        if self._scheduler_initialized:
            raise RuntimeError(
                "Cannot unregister objects after scheduler initialization. "
                "Unregister objects in robotInit() before the scheduler starts."
            )
        if obj in self._telemetry_publishables:
            self._telemetry_publishables.remove(obj)

    @final
    def register_tunable_publishable(self, obj: TunablePublishable) -> None:
        """
        Manually registers a TunablePublishable object to have its tunables managed.
        Must be registered before robotPeriodic() is first called.
        
        :param obj: The TunablePublishable to register.
        :raises RuntimeError: If called after scheduler initialization.
        """
        if self._scheduler_initialized:
            raise RuntimeError(
                "Cannot register objects after scheduler initialization. "
                "Register objects in robotInit() before the scheduler starts."
            )
        if obj not in self._tunable_publishables:
            self._tunable_publishables.append(obj)

    @final
    def unregister_tunable_publishable(self, obj: TunablePublishable) -> None:
        """
        Unregisters a TunablePublishable object. Can only be called before robotPeriodic().
        
        :param obj: The TunablePublishable to unregister.
        :raises RuntimeError: If called after scheduler initialization.
        """
        if self._scheduler_initialized:
            raise RuntimeError(
                "Cannot unregister objects after scheduler initialization. "
                "Unregister objects in robotInit() before the scheduler starts."
            )
        if obj in self._tunable_publishables:
            self._tunable_publishables.remove(obj)

    @final
    def register_schedulable(self, obj: Schedulable) -> None:
        """
        Manually registers a Schedulable object to be scheduled each iteration.
        Must be registered before robotPeriodic() is first called.
        
        :param obj: The Schedulable to register.
        :raises RuntimeError: If called after scheduler initialization.
        """
        if self._scheduler_initialized:
            raise RuntimeError(
                "Cannot register objects after scheduler initialization. "
                "Register objects in robotInit() before the scheduler starts."
            )
        if obj not in self._schedulables:
            self._schedulables.append(obj)

    @final
    def unregister_schedulable(self, obj: Schedulable) -> None:
        """
        Unregisters a Schedulable object. Can only be called before robotPeriodic().
        
        :param obj: The Schedulable to unregister.
        :raises RuntimeError: If called after scheduler initialization.
        """
        if self._scheduler_initialized:
            raise RuntimeError(
                "Cannot unregister objects after scheduler initialization. "
                "Unregister objects in robotInit() before the scheduler starts."
            )
        if obj in self._schedulables:
            self._schedulables.remove(obj)

    @final
    def register_component(self, component: AdaptiveComponent) -> None:
        """
        Manually registers a component (AdaptiveComponent) to be scheduled.
        Components are AdaptiveComponent instances which implement all three interfaces.
        
        :param component: The AdaptiveComponent to register.
        :raises RuntimeError: If called after scheduler initialization.
        """
        self.register_schedulable(component)
        self.register_telemetry_publishable(component)
        self.register_tunable_publishable(component)

    @final
    def unregister_component(self, component: AdaptiveComponent) -> None:
        """
        Unregisters a component (AdaptiveComponent) from being scheduled.
        
        :param component: The AdaptiveComponent to unregister.
        :raises RuntimeError: If called after scheduler initialization.
        """
        self.unregister_schedulable(component)
        self.unregister_telemetry_publishable(component)
        self.unregister_tunable_publishable(component)

    @final
    def get_registered_telemetry_publishables(self) -> list[TelemetryPublishable]:
        """
        Returns all registered TelemetryPublishable interfaces.
        """
        return self._telemetry_publishables

    @final
    def get_registered_tunable_publishables(self) -> list[TunablePublishable]:
        """
        Returns all registered TunablePublishable interfaces.
        """
        return self._tunable_publishables
    
    @final
    def get_registered_schedulables(self) -> list[Schedulable]:
        """
        Returns all registered Schedulable interfaces.
        """
        return self._schedulables

    @final
    def schedule_action(self, action: AsyncAction, name: str) -> str | None:
        """
        Schedules an autonomous action for this robot iteration.
        Only one autonomous action can run at a time.
        If another autonomous action is already running, this call returns None and logs a warning.

        :param action: The AsyncAction routine to run.
        :param name: A descriptive name for the autonomous action.
        :returns: The provided name of the AsyncAction, or None if another autonomous action is already running.
        """
        return self._action_scheduler.schedule_action(action, name)

    @final
    def cancel_action(self, name: str) -> str | None:
        """
        Cancels an AsyncAction if it is currently running in the scheduler.

        :returns: The provided name of the cancelled AsyncAction, or None if the AsyncAction
        was not running.
        """
        return self._action_scheduler.cancel(name)

    @final
    def cancel_all_actions(self) -> list[str] | None:
        """
        Cancels all AsyncActions that are in the scheduler.

        :returns: A list of the provided names of the cancelled AsyncActions, or None if there
        were no AsyncActions to cancel.
        """
        return self._action_scheduler.cancel_all()

    @final
    def action_is_running(self, name: str) -> bool:
        """
        Returns True if an Action with the provided name is currently running.
        An Action is considered running if it has not finished naturally or been cancelled.
        """
        return self._action_scheduler.is_running(name)

    @final
    def get_running_actions(self) -> list[AsyncAction]:
        """
        Returns a list of all the currently running Actions.
        An Action is considered running if it has not finished naturally or been cancelled.
        """
        return self._action_scheduler.get_running_actions()

    @final
    def robotInit(self) -> None:
        """
        Calls the onRobotInit hook.  
        Auto-discovers all interface implementers and initializes all three subschedulers.  
        Sets up the FaultManager to coordinate all schedulers.  
        Marks the scheduler as fully initialized.  
        """
        try:
            self.onRobotInit()
        except Exception as e:
            wpilib.reportError(f"Error in onRobotInit: {e}")

        if self._config.auto_discover_interfaces:
            self._auto_discover_all_interfaces()
        
        # Create subschedulers with discovered objects
        self._telemetry_subscheduler = TelemetrySubscheduler(
            self._telemetry_publisher,
            self._telemetry_struct_publisher,
            self._telemetry_publishables
        )
        
        self._tunable_subscheduler = TunableSubscheduler(
            self._tunable_publishables
        )
        
        self._schedulable_subscheduler = SchedulableSubscheduler(
            self._schedulables, self._config.default_fault_threshold
        )

        self._fault_manager = FaultManager(
            schedulable_subscheduler=self._schedulable_subscheduler,
            telemetry_subscheduler=self._telemetry_subscheduler,
            tunable_subscheduler=self._tunable_subscheduler,
            profiling_subscheduler=self._profiling_subscheduler,
            action_scheduler=self._action_scheduler, 
            fault_logger=self._fault_logger
        )
        
        self._scheduler_initialized = True

    @final
    def robotPeriodic(self) -> None:
        """
        Runs the FaultManager each iteration.  
        Disables the robot on a CRITICAL Fault.
        Runs the profiling subscheduler each iteration.
        Calls the onRobotPeriodic hook each iteration.

        :raises RuntimeError: If the FaultManager was never initialized.
        :raises RobotDisable: Upon a CRITICAL Fault from the FaultManager.
        """
        if self._fault_manager is None:
            raise RuntimeError(
                "FaultManager was not initialized! This should be automatically set up in robotInit(). "
                "Ensure robotInit() is being called by WPILib's TimedRobot base class."
            )

        should_disable = self._fault_manager.run(self.isEnabled())
        if should_disable:
            raise RobotDisable("CRITICAL Fault raised. Robot has been cleanly disabled.")

        try:
            self.onRobotPeriodic()
        except Exception as e:
            wpilib.reportError(f"Error in onRobotPeriodic: {e}")

    @final
    def disabledInit(self) -> None:
        """
        Cancels all Actions, resets all Schedulables to HEALTHY, and calls the onDisabledInit hook.

        :raises RuntimeError: If the SchedulableSubscheduler was never initialized.
        """
        if self._schedulable_subscheduler is None:
            raise RuntimeError(
                "SchedulableSubscheduler was not initialized! This should be automatically set up in robotInit(). "
                "Ensure robotInit() is being called by WPILib's TimedRobot base class and that you're not calling "
                "disabledInit() before robotInit()."
            )

        try:
            self.cancel_all_actions()
        except Exception as e:
            wpilib.reportWarning(f"Error cancelling actions during disabledInit: {e}")

        try:
            self._schedulable_subscheduler.reset_all_schedulable_health()
        except Exception as e:
            wpilib.reportError(f"Error resetting schedulable health during disabledInit: {e}")

        try:
            self.onDisabledInit()
        except Exception as e:
            wpilib.reportError(f"Error in onDisabledInit: {e}")

    @final
    def disabledPeriodic(self) -> None:
        try:
            self.onDisabledPeriodic()
        except Exception as e:
            wpilib.reportError(f"Error in onDisabledPeriodic: {e}")

    @final
    def disabledExit(self) -> None:
        try:
            self.onDisabledExit()
        except Exception as e:
            wpilib.reportError(f"Error in onDisabledExit: {e}")
        finally:
            try:
                TunablePersistenceManager.save_all()
            except Exception as e:
                wpilib.reportWarning(f"Error saving tunables during disabledExit: {e}")

    @final
    def teleopInit(self) -> None:
        try:
            self.onTeleopInit()
        except Exception as e:
            wpilib.reportError(f"Error in onTeleopInit: {e}")

    @final
    def teleopPeriodic(self) -> None:
        try:
            self.onTeleopPeriodic()
        except Exception as e:
            wpilib.reportError(f"Error in onTeleopPeriodic: {e}")

    @final
    def teleopExit(self) -> None:
        try:
            self.onTeleopExit()
        except Exception as e:
            wpilib.reportError(f"Error in onTeleopExit: {e}")

    @final
    def autonomousInit(self) -> None:
        try:
            self.onAutonomousInit()
        except Exception as e:
            wpilib.reportError(f"Error in onAutonomousInit: {e}")

    @final
    def autonomousPeriodic(self) -> None:
        try:
            self.onAutonomousPeriodic()
        except Exception as e:
            wpilib.reportError(f"Error in onAutonomousPeriodic: {e}")

    @final
    def autonomousExit(self) -> None:
        try:
            self.cancel_all_actions()
        except Exception as e:
            wpilib.reportWarning(f"Error cancelling actions during autonomousExit: {e}")

        try:
            self.onAutonomousExit()
        except Exception as e:
            wpilib.reportError(f"Error in onAutonomousExit: {e}")
    
    @final
    def testInit(self) -> None:
        try:
            self.onTestInit()
        except Exception as e:
            wpilib.reportError(f"Error in onTestInit: {e}")

    @final
    def testPeriodic(self) -> None:
        try:
            self.onTestPeriodic()
        except Exception as e:
            wpilib.reportError(f"Error in onTestPeriodic: {e}")

    @final
    def testExit(self) -> None:
        try:
            self.cancel_all_actions()
        except Exception as e:
            wpilib.reportWarning(f"Error cancelling actions during testExit: {e}")

        try:
            self.onTestExit()
        except Exception as e:
            wpilib.reportError(f"Error in onTestExit: {e}")

    def onRobotInit(self) -> None:
        """
        Robot-wide initialization code should go here.

        Users should override this method for default Robot-wide initialization which will be called 
        when the robot is first powered on. 
        It will be called exactly one time.
        """
        pass

    def onRobotPeriodic(self) -> None:
        """
        Periodic code for all modes should go here.

        This function is called each time a new packet is received from the driver station.
        """
        pass
    
    def onDisabledInit(self) -> None:
        """
        Initialization code for disabled mode should go here.

        Users should override this method for initialization code 
        which will be called each time the robot enters disabled mode.
        """
        pass

    def onDisabledPeriodic(self) -> None:
        """
        Periodic code for disabled mode should go here.

        Users should override this method for code which will be called each time a new packet 
        is received from the driver station and the robot is in disabled mode.
        """
        pass

    def onDisabledExit(self) -> None:
        """
        Exit code for disabled mode should go here.

        Users should override this method for code 
        which will be called each time the robot exits disabled mode.
        """
        pass
    
    def onTeleopInit(self) -> None:
        """
        Initialization code for teleop mode should go here.

        Users should override this method for initialization code 
        which will be called each time the robot enters teleop mode.
        """
        pass

    def onTeleopPeriodic(self) -> None:
        """
        Periodic code for teleop mode should go here.

        Users should override this method for code which will be called each time a new packet 
        is received from the driver station and the robot is in teleop mode.
        """
        pass

    def onTeleopExit(self) -> None:
        """
        Exit code for teleop mode should go here.

        Users should override this method for code 
        which will be called each time the robot exits teleop mode.
        """
        pass

    def onAutonomousInit(self) -> None:
        """
        Initialization code for autonomous mode should go here.

        Users should override this method for initialization code 
        which will be called each time the robot enters autonomous mode.
        """
        pass

    def onAutonomousPeriodic(self) -> None:
        """
        Periodic code for autonomous mode should go here.

        Users should override this method for code which will be called each time a new packet 
        is received from the driver station and the robot is in autonomous mode."""
        pass
    
    def onAutonomousExit(self) -> None:
        """
        Exit code for autonomous mode should go here.

        Users should override this method for code 
        which will be called each time the robot exits autonomous mode.
        """
        pass

    def onTestInit(self) -> None:
        """
        Initialization code for test mode should go here.

        Users should override this method for initialization code 
        which will be called each time the robot enters test mode.
        """
        pass

    def onTestPeriodic(self) -> None:
        """
        Periodic code for test mode should go here.

        Users should override this method for code which will be called each time a new packet 
        is received from the driver station and the robot is in test mode.
        """
        pass

    def onTestExit(self) -> None:
        """
        Exit code for test mode should go here.

        Users should override this method for code 
        which will be called each time the robot exits test mode.
        """
        pass
