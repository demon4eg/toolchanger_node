# Command from host (write) or status from ESP32 (read)
uint8 command           # 0=release, 1=attach, 2=status_request
uint8 tool_id           # Tool number (1-255, 0=none)
uint8 state             # 0=idle, 1=attaching, 2=attached, 3=releasing, 4=error
uint8 error_code        # 0=ok, 1=wrong_tool, 2=timeout, 3=no_tool
float32 temperature     # DS18B20 reading