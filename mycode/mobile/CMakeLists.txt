cmake_minimum_required(VERSION 3.20.0)
   find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
   project(hello_world)

   # Collect all the source files in src
   FILE(GLOB app_sources src/*.c)

   # Collect all the source files in lib
   FILE(GLOB lib_sources ../lib/*.c)

   # Tell CMake to build with the app and lib sources
   target_sources(app PRIVATE ${app_sources} ${lib_sources})

   # Tell CMake where our header files are
   zephyr_include_directories(../inc)
