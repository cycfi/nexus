cmake_minimum_required(VERSION 3.14)
project(nexus_midi C CXX)

# ---------------------------------------------------------------------------
# Energia SDK locations (installed by Energia.app's board manager).
# ---------------------------------------------------------------------------
if(NOT DEFINED ENERGIA_ROOT)
   set(ENERGIA_ROOT "$ENV{HOME}/Library/Energia15/packages/energia")
endif()
set(EN_MSP           "${ENERGIA_ROOT}/hardware/msp430/1.0.7")
set(MSP430_TOOLCHAIN "${ENERGIA_ROOT}/tools/msp430-gcc/4.6.6")
set(CORE     "${EN_MSP}/cores/msp430")
set(VARIANT  "${EN_MSP}/variants/MSP-EXP430G2553LP")
set(MSPFLASH "${EN_MSP}/libraries/MspFlash")

# ---------------------------------------------------------------------------
# Board build settings (from boards.txt / platform.txt).
# ---------------------------------------------------------------------------
# Inject synthetic ADC test data through the raw_adc() seam (HIL stress test).
option(NEXUS_SIM "Feed synthetic ADC data into the controls" OFF)
if(NEXUS_SIM)
   add_compile_definitions(NEXUS_SIM)
endif()

set(IDE_VERSION 10811)               # Energia 1.8.11 -> runtime.ide.version
add_compile_definitions(
   F_CPU=16000000L
   ARDUINO=${IDE_VERSION}
   ENERGIA=${IDE_VERSION}
   ENERGIA_MSP_EXP430G2553LP         # -DENERGIA_{build.board}
   ENERGIA_ARCH_MSP430               # -DENERGIA_ARCH_{build.arch}
)

# Plain -I (not SYSTEM/-isystem): with -isystem GCC treats the core as a
# system-header tree and an avr/* header leaves an extern "C" open when
# WString.h is reached, breaking the C++ operator overloads.
include_directories(
   ${CMAKE_CURRENT_SOURCE_DIR}    # the sketch's own midi.hpp / util.hpp
   ${CORE}
   ${VARIANT}
   ${MSPFLASH}
   ${MSP430_TOOLCHAIN}/include
)

# ---------------------------------------------------------------------------
# Common compile flags (mirror platform.txt recipes).
# ---------------------------------------------------------------------------
add_compile_options(-Os -w -ffunction-sections -fdata-sections)
add_compile_options(
   $<$<COMPILE_LANGUAGE:CXX>:-std=c++0x>
   $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
   $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>
)

# ---------------------------------------------------------------------------
# Energia core + MspFlash, built as a static library (-> core archive).
# ---------------------------------------------------------------------------
file(GLOB CORE_SOURCES
   "${CORE}/*.c"
   "${CORE}/*.cpp"
   "${CORE}/avr/*.c"
)
add_library(energia_core STATIC ${CORE_SOURCES} "${MSPFLASH}/MspFlash.cpp")

# ---------------------------------------------------------------------------
# Sketch: generate a .cpp from the .ino so the repo keeps its .ino for the IDE.
# ---------------------------------------------------------------------------
set(SKETCH_INO "${CMAKE_CURRENT_SOURCE_DIR}/nexus_midi.ino")
set(SKETCH_CPP "${CMAKE_CURRENT_BINARY_DIR}/nexus_midi.cpp")
add_custom_command(
   OUTPUT  "${SKETCH_CPP}"
   COMMAND "${CMAKE_COMMAND}" -DINO=${SKETCH_INO} -DOUT=${SKETCH_CPP}
           -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_sketch.cmake"
   DEPENDS "${SKETCH_INO}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_sketch.cmake"
   COMMENT "Generating nexus_midi.cpp from nexus_midi.ino"
   VERBATIM
)

# ---------------------------------------------------------------------------
# Firmware ELF.  Link with msp430-gcc (LINKER_LANGUAGE C) like Energia does,
# to avoid pulling in libstdc++.
# ---------------------------------------------------------------------------
add_executable(nexus_midi.elf "${SKETCH_CPP}")
set_target_properties(nexus_midi.elf PROPERTIES LINKER_LANGUAGE C)
target_link_libraries(nexus_midi.elf energia_core m)
target_link_options(nexus_midi.elf PRIVATE
   -Os -fno-rtti -fno-exceptions
   -Wl,--gc-sections,-u,main
   "-L${MSP430_TOOLCHAIN}/include"
)

# ---------------------------------------------------------------------------
# .hex output + size report.
# ---------------------------------------------------------------------------
set(MSP430_OBJCOPY "${MSP430_TOOLCHAIN}/bin/msp430-objcopy")
set(MSP430_SIZE    "${MSP430_TOOLCHAIN}/bin/msp430-size")
add_custom_command(TARGET nexus_midi.elf POST_BUILD
   COMMAND "${MSP430_OBJCOPY}" -O ihex -R .eeprom
           "$<TARGET_FILE:nexus_midi.elf>" "${CMAKE_CURRENT_BINARY_DIR}/nexus_midi.hex"
   COMMAND "${MSP430_SIZE}" -A "$<TARGET_FILE:nexus_midi.elf>"
   COMMENT "Creating nexus_midi.hex + size report"
)

# ---------------------------------------------------------------------------
# Flash to a classic MSP430G2 LaunchPad (onboard FET = mspdebug rf2500 driver).
#   cmake --build build --target upload
# ---------------------------------------------------------------------------
add_custom_target(upload
   COMMAND mspdebug rf2500 --force-reset "prog ${CMAKE_CURRENT_BINARY_DIR}/nexus_midi.hex"
   DEPENDS nexus_midi.elf
   USES_TERMINAL
)
