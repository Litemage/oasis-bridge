cmake_minimum_required(VERSION 3.10)

set(PROJECT_INCLUDE_DIR ./include)

project(oasis-bridge)

# Add executable with sources
add_executable(oasis-bridge ./src/main.c)

# Linux I2C
target_link_libraries(oasis-bridge i2c)

# C Redis Client
target_link_libraries(oasis-bridge hiredis)

# C math
target_link_libraries(oasis-bridge m)

# Set include directory
target_include_directories(oasis-bridge PUBLIC "${PROJECT_INCLUDE_DIR}")
target_include_directories(oasis-bridge PUBLIC "${PROJECT_INCLUDE_DIR}")
