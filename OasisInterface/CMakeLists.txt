cmake_minimum_required(VERSION 3.10)

set(PROJECT_INCLUDE_DIR ./include)

project(oasis-interface)

# Add executable with sources
add_executable(oasis-interface ./src/main.c)

target_link_libraries(oasis-interface i2c)

# Set include directory
target_include_directories(oasis-interface PUBLIC "${PROJECT_INCLUDE_DIR}")
target_include_directories(oasis-interface PUBLIC "${PROJECT_INCLUDE_DIR}")
