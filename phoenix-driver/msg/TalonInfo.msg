std_msgs/Header header

float64 position
float64 velocity
float64 acceleration

float32 device_temp
float32 processor_temp
float32 bus_voltage
float32 supply_current

float32 output_percent
float32 output_voltage
float32 output_current

uint8 motor_state
uint8 bridge_mode
uint8 control_mode

uint8 STATUS_ROS_ENABLED = 1
uint8 STATUS_PHX_ENABLED = 2
uint8 STATUS_CONNECTED = 4
uint8 STATUS_RESET_OCC = 8
uint8 status
