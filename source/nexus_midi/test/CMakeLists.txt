# Host-side test harness for nexus_midi. Standalone (host compiler) — do NOT
# configure this with the MSP430 toolchain file.
#
#   cmake -S . -B build
#   cmake --build build
#   ./build/host_sim            # decode the synthetic-stimulus MIDI stream
#
cmake_minimum_required(VERSION 3.14)
project(nexus_host_sim CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(host_sim host_sim.cpp)

# NEXUS_HOST guards the device-only .init3 hook; NEXUS_SIM routes raw_adc() to
# the synthetic generator.
target_compile_definitions(host_sim PRIVATE NEXUS_HOST NEXUS_SIM)

# The firmware uses uint8_t{...} brace-init whose narrowing the device's GCC 4.6
# accepts; match that leniency so the harness compiles the firmware as-is.
target_compile_options(host_sim PRIVATE -Wno-c++11-narrowing)

target_include_directories(host_sim PRIVATE
   ${CMAKE_CURRENT_SOURCE_DIR}/mock     # energia.h, MspFlash.h
   ${CMAKE_CURRENT_SOURCE_DIR}/..        # nexus_midi.ino, nexus_controls.hpp, sim/
)

enable_testing()
add_test(NAME host_sim COMMAND host_sim 200 -q)
