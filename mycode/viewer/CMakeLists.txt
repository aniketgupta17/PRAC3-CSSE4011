# cmake_minimum_required(VERSION 3.20.0)
# find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
# project(prac2)

# FILE(GLOB app_sources src/*.c)

# FILE(GLOB lib_sources ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/*.c)

# target_sources(app PRIVATE ${app_sources} ${lib_sources})

# target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include)

# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(prac-3)
FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE src/main.c)


