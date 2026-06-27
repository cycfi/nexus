# CMake toolchain for the Energia-bundled MSP430 GCC (msp430-gcc 4.6.3).
#
# Use:  cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/msp430_toolchain.cmake
#
# Override ENERGIA_ROOT or MSP430_MCU on the command line if your Energia
# install lives elsewhere or you target a different value-line part.

set(CMAKE_SYSTEM_NAME Generic)        # bare-metal, no host OS
set(CMAKE_SYSTEM_PROCESSOR msp430)

if(NOT DEFINED ENERGIA_ROOT)
   set(ENERGIA_ROOT "$ENV{HOME}/Library/Energia15/packages/energia")
endif()
set(MSP430_TOOLCHAIN "${ENERGIA_ROOT}/tools/msp430-gcc/4.6.6")
set(MSP430_GCC_BIN   "${MSP430_TOOLCHAIN}/bin")

if(NOT EXISTS "${MSP430_GCC_BIN}/msp430-gcc")
   message(FATAL_ERROR
      "msp430-gcc not found under ${MSP430_GCC_BIN}.\n"
      "Set -DENERGIA_ROOT=<path to .../packages/energia>.")
endif()

set(CMAKE_C_COMPILER   "${MSP430_GCC_BIN}/msp430-gcc")
set(CMAKE_CXX_COMPILER "${MSP430_GCC_BIN}/msp430-g++")

set(MSP430_MCU "msp430g2553" CACHE STRING "Target MSP430 MCU")

# The MCU flag is needed at both compile and link time; seed the language
# flags so it propagates to every invocation (CMake puts <LANG>_FLAGS on the
# link line too).
set(CMAKE_C_FLAGS_INIT   "-mmcu=${MSP430_MCU}")
set(CMAKE_CXX_FLAGS_INIT "-mmcu=${MSP430_MCU}")

# Probe the compiler by building an object only — a full link would need the
# device ldscript and would fail the compiler-id check.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH "${MSP430_TOOLCHAIN}")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
