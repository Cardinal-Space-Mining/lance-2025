std_msgs/Header header

uint32 faults
uint32 sticky_faults

bool hardware_fault
bool proc_temp_fault
bool device_temp_fault
bool undervoltage_fault
bool boot_fault              # "boot while enabled"
bool unliscensed_fault       # "use of unliscensed feature"
bool bridge_brownout_fault
bool overvoltage_fault
bool unstable_voltage_fault
bool stator_current_limit_fault
bool supply_current_limit_fault
bool static_brake_disabled_fault

bool sticky_hardware_fault
bool sticky_proc_temp_fault
bool sticky_device_temp_fault
bool sticky_undervoltage_fault
bool sticky_boot_fault              # "boot while enabled"
bool sticky_unliscensed_fault       # "use of unliscensed feature"
bool sticky_bridge_brownout_fault
bool sticky_overvoltage_fault
bool sticky_unstable_voltage_fault
bool sticky_stator_current_limit_fault
bool sticky_supply_current_limit_fault
bool sticky_static_brake_disabled_fault
