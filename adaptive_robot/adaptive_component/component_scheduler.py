import logging

import wpilib

from adaptive_robot.adaptive_component.adaptive_component import AdaptiveComponent, ComponentContext
from adaptive_robot.telemetry.telemetry import TelemetryPublisher
from adaptive_robot.telemetry.struct_telemetry import TelemetryStructPublisher
from adaptive_robot.faults.faults import FaultSeverity, FaultException
from adaptive_robot.interfaces.subscheduler import Subscheduler


logger = logging.getLogger(__name__)


class ComponentScheduler(Subscheduler):
    """
    Subscheduler that updates all components each iteration.
    Manages component execution, health tracking, and telemetry publishing.
    """
    def __init__(
        self,
        components: list[AdaptiveComponent],
        telemetry_publisher: TelemetryPublisher,
        struct_telemetry_publisher: TelemetryStructPublisher,
        fault_threshold: float = 10
    ) -> None:
        """
        Registers all components into the scheduler and attaches them to a robot.

        :raises ValueError: If fault_threshold is negative or zero.
        """
        if fault_threshold <= 0:
            raise ValueError(
                f"fault_threshold must be positive, got {fault_threshold}. "
                f"This parameter controls how many consecutive faults before a component is marked unhealthy."
            )
        
        self.components = components
        self.telemetry_publisher = telemetry_publisher
        self.struct_telemetry_publisher = struct_telemetry_publisher
        self.fault_threshold = fault_threshold

        self.context = ComponentContext(
            telemetry=telemetry_publisher,
            struct_telemetry=struct_telemetry_publisher,
            logger=logger
        )

        # Track consecutive faults per component for health management
        self._consecutive_faults: dict[AdaptiveComponent, int] = {component: 0 for component in components}

        for component in components:
            component.context = self.context
        
        self.previously_enabled = None

    def _record_component_fault(self, component: AdaptiveComponent) -> None:
        """
        Records an ERROR fault for a component and marks it unhealthy if threshold is reached.

        :raises FaultException: If on_faulted_init raises an Exception.
        """
        self._consecutive_faults[component] += 1
        
        if self._consecutive_faults[component] >= self.fault_threshold:
            wpilib.reportWarning(
                f"Component {component.__class__.__name__} reached consecutive fault threshold ({self.fault_threshold}). "
                f"Marking component as unhealthy (faulted)."
            )
            component.set_health(False)
            try:
                component.on_faulted_init()
            except Exception as e:
                message = f"Error occured when calling on_faulted_init: {e}"
                self.raise_fault(component, FaultSeverity.CRITICAL, message, e)

    def _reset_component_faults(self, component: AdaptiveComponent) -> None:
        """
        Resets fault counter for a component after successful execution per iteration.
        """
        self._consecutive_faults[component] = 0

    def _update_tunables(self) -> None:
        """
        Updates all tunable values and PID controllers.

        :raises FaultException: Upon an Exception being raised by tunables.
        """
        try:
            for tunable in self.context.tunables:
                tunable.update()
            for tunablePID in self.context.tunable_pids:
                tunablePID.update_from_tunables()
        except FaultException:
            raise
        except Exception as e:
            message = f"Tunable update error: {e}"
            self.raise_fault(None, FaultSeverity.WARNING, message, e)

    def _publish_telemetry(self) -> None:
        """
        Publishes telemetry for all components.

        :raises FaultException: Upon publish_telemetry raising an Exception.
        """
        for component in self.components:
            try:
                component.publish_telemetry()
            except FaultException:
                raise
            except Exception as e:
                message = f"Telemetry error for {component.__class__.__name__}: {e}"
                self.raise_fault(component, FaultSeverity.WARNING, message, e)

    def _execute_components(self, enabled: bool) -> None:
        """
        Executes healthy components or calls on_faulted() for unhealthy components if enabled.  
        Tracks consecutive faults and automatically marks components unhealthy after
        threshold if faulted.

        :raises FaultException: Upon execute() or on_faulted_periodic() raising an Exception.
        """
        for component in self.components:
            if component.locked:
                continue
            try:
                if enabled:
                    if component.is_healthy():
                        component.execute()
                        # Reset fault counter on successful execution
                        self._reset_component_faults(component)
                    else:
                        component.on_faulted_periodic()
            except FaultException:
                self._record_component_fault(component)
                raise
            except Exception as e:
                message = f"CRITICAL {component.__class__.__name__} raised non-FaultException error: {e}"
                self._record_component_fault(component)
                self.raise_fault(component, FaultSeverity.CRITICAL, message, e)

    def _call_activation_methods(self, enabled: bool) -> None:
        """
        Calls the activation methods (on_enabled and on_disabled) upon the robot
        having just enabled or disabled.

        :raises FaultException: Upon on_enabled() raising.
        """
        # If switch from disabled to enabled
        if not self.previously_enabled and enabled:
            for component in self.components:
                if component.locked:
                    continue
                try:
                    component.on_enabled()
                except FaultException as e:
                    self._record_component_fault(component)
                    raise
                except Exception as e:
                    message = f"CRITICAL {component.__class__.__name__} raised non-FaultException error: {e}"
                    self._record_component_fault(component)
                    self.raise_fault(component, FaultSeverity.CRITICAL, message, e)
        
        # If switch from enabled to disabled
        if self.previously_enabled and not enabled:
            for component in self.components:
                if component.locked:
                    continue
                try:
                    component.on_disabled()
                
                except FaultException as e:
                    self._record_component_fault(component)
                    raise
                except Exception as e:
                    message = f"CRITICAL {component.__class__.__name__} raised non-FaultException error: {e}"
                    self._record_component_fault(component)
                    self.raise_fault(component, FaultSeverity.CRITICAL, message, e)

    def run(self, enabled: bool) -> None:
        """
        Executes the subscheduler each iteration if enabled.
        Updates tunable values and PID controllers, publishes telemetry, 
        executes healthy components, and handles faults regardless of being enabled or not.

        All telemetry and tunable Faults are treated as WARNING - they should not crash the robot.
        Component-specific Faults raised will not crash the robot immediately unless CRITICAL.
        Any other Exceptions raised by the component will crash the robot (CRITICAL Faults).

        :raises FaultException: Upon any error when executing state and schedulers.
        """
        try:
            self._update_tunables()
            self._publish_telemetry()
            self._call_activation_methods(enabled)
            self._execute_components(enabled)
        finally:
            self.previously_enabled = enabled
    
    def reset_all_component_health(self) -> None:
        """
        Resets all components to HEALTHY and clears their fault counters.
        Called when the robot is disabled to prepare for next match.
        """
        for component in self.components:
            component.set_health(True)
            self._consecutive_faults[component] = 0
