add_executable(example1
  example1.cpp
)
target_link_libraries(example1
  PRIVATE l6470-linux
)

add_executable(example2
  example2.cpp
)
target_link_libraries(example2
  PRIVATE l6470-linux
)
