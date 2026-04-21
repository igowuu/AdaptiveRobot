from adaptive_robot import Faultable, FaultSeverity

from phoenix6.hardware import TalonFX


class TalonFXFaultLogger(Faultable):
    """
    Handles and reports all TalonFX faults, raising them as Fault objects.
    """
    FAULT_DEFINITIONS = [
        ('get_fault_supply_curr_limit', 'SUPPLY_CURRENT_LIMIT', FaultSeverity.ERROR),
        ('get_fault_boot_during_enable', 'BOOT_DURING_ENABLE', FaultSeverity.ERROR),
        ('get_fault_bridge_brownout', 'BRIDGE_BROWNOUT', FaultSeverity.ERROR),
        ('get_fault_device_temp', 'DEVICE_TEMP', FaultSeverity.ERROR),
        ('get_fault_field', 'FIELD', FaultSeverity.ERROR),
        ('get_fault_forward_hard_limit', 'FORWARD_HARD_LIMIT', FaultSeverity.ERROR),
        ('get_fault_forward_soft_limit', 'FORWARD_SOFT_LIMIT', FaultSeverity.ERROR),
        ('get_fault_fused_sensor_out_of_sync', 'FUSED_SENSOR_OUT_OF_SYNC', FaultSeverity.ERROR),
        ('get_fault_hardware', 'HARDWARE', FaultSeverity.ERROR),
        ('get_fault_missing_differential_fx', 'MISSING_DIFFERENTIAL_FX', FaultSeverity.ERROR),
        ('get_fault_missing_hard_limit_remote', 'MISSING_HARD_LIMIT_REMOTE', FaultSeverity.ERROR),
        ('get_fault_missing_soft_limit_remote', 'MISSING_SOFT_LIMIT_REMOTE', FaultSeverity.ERROR),
        ('get_fault_over_supply_v', 'OVER_SUPPLY_VOLTAGE', FaultSeverity.ERROR),
        ('get_fault_proc_temp', 'PROCESSOR_TEMP', FaultSeverity.ERROR),
        ('get_fault_remote_sensor_data_invalid', 'REMOTE_SENSOR_DATA_INVALID', FaultSeverity.ERROR),
        ('get_fault_remote_sensor_pos_overflow', 'REMOTE_SENSOR_POS_OVERFLOW', FaultSeverity.ERROR),
        ('get_fault_remote_sensor_reset', 'REMOTE_SENSOR_RESET', FaultSeverity.ERROR),
        ('get_fault_reverse_hard_limit', 'REVERSE_HARD_LIMIT', FaultSeverity.ERROR),
        ('get_fault_reverse_soft_limit', 'REVERSE_SOFT_LIMIT', FaultSeverity.ERROR),
        ('get_fault_static_brake_disabled', 'STATIC_BRAKE_DISABLED', FaultSeverity.ERROR),
        ('get_fault_stator_curr_limit', 'STATOR_CURRENT_LIMIT', FaultSeverity.ERROR),
        ('get_fault_undervoltage', 'UNDERVOLTAGE', FaultSeverity.ERROR),
        ('get_fault_unlicensed_feature_in_use', 'UNLICENSED_FEATURE_IN_USE', FaultSeverity.ERROR),
        ('get_fault_unstable_supply_v', 'UNSTABLE_SUPPLY_VOLTAGE', FaultSeverity.ERROR),
    ]

    STICKY_FAULT_DEFINITIONS = [
        ('get_sticky_fault_supply_curr_limit', 'SUPPLY_CURRENT_LIMIT', FaultSeverity.ERROR),
        ('get_sticky_fault_boot_during_enable', 'BOOT_DURING_ENABLE', FaultSeverity.ERROR),
        ('get_sticky_fault_bridge_brownout', 'BRIDGE_BROWNOUT', FaultSeverity.ERROR),
        ('get_sticky_fault_device_temp', 'DEVICE_TEMP', FaultSeverity.ERROR),
        ('get_sticky_fault_field', 'FIELD', FaultSeverity.ERROR),
        ('get_sticky_fault_forward_hard_limit', 'FORWARD_HARD_LIMIT', FaultSeverity.ERROR),
        ('get_sticky_fault_forward_soft_limit', 'FORWARD_SOFT_LIMIT', FaultSeverity.ERROR),
        ('get_sticky_fault_fused_sensor_out_of_sync', 'FUSED_SENSOR_OUT_OF_SYNC', FaultSeverity.ERROR),
        ('get_sticky_fault_hardware', 'HARDWARE', FaultSeverity.ERROR),
        ('get_sticky_fault_missing_differential_fx', 'MISSING_DIFFERENTIAL_FX', FaultSeverity.ERROR),
        ('get_sticky_fault_missing_hard_limit_remote', 'MISSING_HARD_LIMIT_REMOTE', FaultSeverity.ERROR),
        ('get_sticky_fault_missing_soft_limit_remote', 'MISSING_SOFT_LIMIT_REMOTE', FaultSeverity.ERROR),
        ('get_sticky_fault_over_supply_v', 'OVER_SUPPLY_VOLTAGE', FaultSeverity.ERROR),
        ('get_sticky_fault_proc_temp', 'PROCESSOR_TEMP', FaultSeverity.ERROR),
        ('get_sticky_fault_remote_sensor_data_invalid', 'REMOTE_SENSOR_DATA_INVALID', FaultSeverity.ERROR),
        ('get_sticky_fault_remote_sensor_pos_overflow', 'REMOTE_SENSOR_POS_OVERFLOW', FaultSeverity.ERROR),
        ('get_sticky_fault_remote_sensor_reset', 'REMOTE_SENSOR_RESET', FaultSeverity.ERROR),
        ('get_sticky_fault_reverse_hard_limit', 'REVERSE_HARD_LIMIT', FaultSeverity.ERROR),
        ('get_sticky_fault_reverse_soft_limit', 'REVERSE_SOFT_LIMIT', FaultSeverity.ERROR),
        ('get_sticky_fault_static_brake_disabled', 'STATIC_BRAKE_DISABLED', FaultSeverity.ERROR),
        ('get_sticky_fault_stator_curr_limit', 'STATOR_CURRENT_LIMIT', FaultSeverity.ERROR),
        ('get_sticky_fault_undervoltage', 'UNDERVOLTAGE', FaultSeverity.ERROR),
        ('get_sticky_fault_unlicensed_feature_in_use', 'UNLICENSED_FEATURE_IN_USE', FaultSeverity.ERROR),
        ('get_sticky_fault_unstable_supply_v', 'UNSTABLE_SUPPLY_VOLTAGE', FaultSeverity.ERROR),
    ]

    def report_talonfx_faults(self, motor: TalonFX, sticky: bool = False, name: str = "Unknown") -> None:
        """
        Checks and reports all TalonFX faults.

        :param motor: The TalonFX motor to check.
        :param sticky: If True, checks sticky faults (persist until cleared). If False, checks current faults.
        :param name: Motor name for logging.
        """
        fault_defs = self.STICKY_FAULT_DEFINITIONS if sticky else self.FAULT_DEFINITIONS

        for method_name, display_name, severity in fault_defs:
            try:
                fault_method = getattr(motor, method_name)
                if fault_method():
                    fault_type = "sticky fault" if sticky else "fault"
                    self.raise_fault(
                        None,
                        severity,
                        f"TalonFX {fault_type} [{display_name}] on motor '{name}'"
                    )
            except AttributeError:
                raise
            except Exception as e:
                self.raise_fault(
                    None,
                    FaultSeverity.WARNING,
                    f"Error checking TalonFX fault [{display_name}] on motor '{name}': {e}"
                )
