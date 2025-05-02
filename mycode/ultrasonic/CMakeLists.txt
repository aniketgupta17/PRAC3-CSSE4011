cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ultrasonic_node)

# Collect all .c files in this app's src/ and lib/ directories
file(GLOB app_sources ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)
file(GLOB lib_sources ${CMAKE_CURRENT_SOURCE_DIR}/lib/*.c)

# Add all sources to the app target
target_sources(app PRIVATE ${app_sources} ${lib_sources})

# Include this app's include/ and lib/ directories for header files
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
)
