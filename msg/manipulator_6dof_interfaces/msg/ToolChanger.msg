# Command from host (write) or status from ESP32 (read)
uint8 command           # 0=release, 1=attach, 2=status_request
uint16 tool_id          # Tool number (0-65535) - uses last 2 bytes of DS18B20
uint8 state             # 0=idle, 1=attaching, 2=attached, 3=releasing, 4=error
uint8 error_code        # 0=ok, 1=wrong_tool, 2=timeout, 3=no_tool
float32 temperature     # DS18B20 reading

# Tool identification and feedback
# tool_changer/tool_type.py or in your node
# TOOL_TYPE_NONE = 0
# TOOL_TYPE_GRIPPER = 1
# TOOL_TYPE_DISPENSER = 2
# TOOL_TYPE_VACUUM = 3
# TOOL_TYPE_UART = 4
uint8 tool_type          # 0=none, 1=gripper, 2=dispenser, 3=vacuum, 4=uart
uint16 tool_position     # Gripper: 0-20000 microns, Dispenser: 0-100%, etc.
uint16 tool_effort       # Gripper: 0-1200 mA, others: 0-100%
uint8 tool_status        # 0=idle, 1=moving, 2=stalled, 3=error