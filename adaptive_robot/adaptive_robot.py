# AdaptiveRobot - FRC architecture
# Copyright (c) 2026 Jacob Taylor (igowu) <https://github.com/igowuu>
# Code from: <https://github.com/igowuu/AdaptiveRobot.git>

# Licensed under the MIT License.
# See https://opensource.org/licenses/MIT for details.

from typing import final

import wpilib

from wpimath.units import seconds

from adaptive_robot.telemetry.telemetry import TelemetryPublisher
from adaptive_robot.telemetry.struct_telemetry import TelemetryStructPublisher
from adaptive_robot.adaptive_component.adaptive_component import AdaptiveComponent
from adaptive_robot.adaptive_component.component_scheduler import ComponentScheduler
from adaptive_robot.autonomous.async_actions import AsyncAction
from adaptive_robot.autonomous.action_scheduler import ActionScheduler
from adaptive_robot.faults.fault_scheduler import FaultLogger
from adaptive_robot.fault_manager import FaultManager


class RobotDisable(Exception):
    """
    Custom exception to indicate that the robot was intentionally disabled.
    """
    def __init__(self, description: str) -> None:
        super().__init__(description)


class AdaptiveRobot(wpilib.TimedRobot):
    """
    Wrapper for TimedRobot that allows for a dynamic structure while enforcing safety measures 
    and quality-of-life improvements.  

    Users should inherit from this to declare a robot and must call the parent constructor.
    """
    def __init__(
        self, 
        period: seconds = 0.02,
        fault_logging_folder: str = "faults/"
    ) -> None:
        """
        Creates an AdaptiveRobot, which schedules components each iteration and
        enables the execution of all other Adaptive features.

        :param period: The amount of seconds each robot loop takes.
        :param fault_logging_folder: Folder to save JSON Fault entries for each run of the codebase.
        """
        super().__init__(period)

        self._telemetry_publisher = TelemetryPublisher()
        self._telemetry_struct_publisher = TelemetryStructPublisher()

        self._action_scheduler = ActionScheduler()

        self._fault_logger = FaultLogger(fault_logging_folder)

        self._components: list[AdaptiveComponent] = []
        self._component_scheduler: ComponentScheduler | None = None

        self._fault_manager: FaultManager | None = None

        self._scheduler_initialized = False

    def _auto_register_components(self) -> None:
        """
        Automatically discovers and registers all AdaptiveComponents into the scheduler.  
        This is called before the ComponentScheduler initializes, so components
        created before robotPeriodic() are automatically scheduled without manual add_component() calls.
        """
        for attr_value in self.__dict__.values():
            if isinstance(attr_value, AdaptiveComponent) and attr_value not in self._components:
                self._components.append(attr_value)

    @final
    def register_component(self, component: AdaptiveComponent) -> None:
        """
        Manually registers a component to be scheduled.  
        Components must be registered before robotPeriodic() is first called.  
        If the specified component is already registered into the scheduler,
        the call will be ignored.  
        
        :param component: The AdaptiveComponent to register.
        :raises RuntimeError: If called after scheduler initialization.
        """
        if self._scheduler_initialized:
            raise RuntimeError(
                "Cannot register components after scheduler initialization. "
                "Register components in robotInit() before the scheduler starts."
            )
        if component not in self._components:
            self._components.append(component)

    @final
    def unregister_component(self, component: AdaptiveComponent) -> None:
        """
        Unregisters a component from being scheduled.  
        Components can only be unregistered before robotPeriodic() is first called.  
        If the specified component is not registered into the scheduler,
        the call will be ignored.  
        
        :param component: The AdaptiveComponent to unregister.
        :raises RuntimeError: If called after scheduler initialization.
        """
        if self._scheduler_initialized:
            raise RuntimeError(
                "Cannot unregister components after scheduler initialization. "
                "Unregister components in robotInit() before the scheduler starts."
            )
        if component in self._components:
            self._components.remove(component)

    @final
    def schedule_action(self, action: AsyncAction, name: str) -> str | None:
        """
        Schedules an AsyncAction to the scheduler to manage manage its 
        lifecycle until finished or cancelled.

        :returns: The provided name of the AsyncAction, or None if the AsyncAction 
        had already been running.
        """
        return self._action_scheduler.schedule(action, name)

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
    def robotInit(self) -> None:
        """
        Calls the onRobotInit hook.  
        Auto-discovers components and sets up the ComponentScheduler.  
        Marks the scheduler as fully initialized.  
        """
        self.onRobotInit()
        self._auto_register_components()

        self._component_scheduler = ComponentScheduler(
            self._components,
            self._telemetry_publisher,
            self._telemetry_struct_publisher
        )
        self._fault_manager = FaultManager(
            component_scheduler=self._component_scheduler, 
            action_scheduler=self._action_scheduler, 
            fault_logger=self._fault_logger
        )
        self._scheduler_initialized = True

    @final
    def robotPeriodic(self) -> None:
        """
        Runs the FaultManager each iteration.  
        Disables the robot on a CRITICAL Fault.
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

        self.onRobotPeriodic()

    @final
    def disabledInit(self) -> None:
        """
        Cancels all Actions, if there are any.
        Resets all components to HEALTHY.
        Calls the onDisabledInit hook.

        :raises RuntimeError: If the ComponentScheduler was never initialized.
        """
        if self._component_scheduler is None:
            raise RuntimeError(
                "ComponentScheduler was not initialized! This should be automatically set up in robotInit(). "
                "Ensure robotInit() is being called by WPILib's TimedRobot base class and that you're not calling "
                "disabledInit() before robotInit()."
            )

        try:
            self.cancel_all_actions()
        except Exception as e:
            wpilib.reportWarning(f"Error cancelling actions during disabledInit: {e}")

        try:
            self._component_scheduler.reset_all_component_health()
        except Exception as e:
            wpilib.reportError(f"Error resetting component health during disabledInit: {e}")

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
