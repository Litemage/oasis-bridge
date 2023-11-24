cmake_minimum_required(VERSION 3.10)

set(PROJECT_INCLUDE_DIR ./include)

project(oasis-interface)

# Add executable with sources
add_executable(oasis-interface ./src/main.c)

# Linux I2C
target_link_libraries(oasis-interface i2c)

# C Redis Client
target_link_libraries(oasis-interface hiredis)

# Set include directory
target_include_directories(oasis-interface PUBLIC "${PROJECT_INCLUDE_DIR}")
target_include_directories(oasis-interface PUBLIC "${PROJECT_INCLUDE_DIR}")
