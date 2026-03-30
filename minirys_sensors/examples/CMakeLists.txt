# Simple temperature example using the MCP9808 sensor
add_executable(example_temperature
  example_temperature.cpp
)
target_link_libraries(example_temperature
  PRIVATE minirys_sensors
)

# Simple IMU example using the LSM6 sensor
add_executable(example_imu
  example_imu.cpp
)
target_link_libraries(example_imu
  PRIVATE minirys_sensors
)

# Simple ADC example using the MAX11613
add_executable(example_adc
  example_adc.cpp
)
target_link_libraries(example_adc
  PRIVATE minirys_sensors
)

# Simple ADC example using the MAX11613
add_executable(example_minirys
  example_minirys.cpp
)
target_link_libraries(example_minirys
  PRIVATE minirys_sensors
)
