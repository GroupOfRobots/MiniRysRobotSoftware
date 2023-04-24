add_executable(exampleGPIO
  exampleGPIO.cpp
)
target_link_libraries(exampleGPIO
  PRIVATE sbc-linux-interfaces
)

add_executable(exampleI2C
  exampleI2C.cpp
)
target_link_libraries(exampleI2C
  PRIVATE sbc-linux-interfaces
)
