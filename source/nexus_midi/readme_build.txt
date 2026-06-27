Building nexus_midi (CMake + Energia MSP430 toolchain)
======================================================

This builds the MSP430G2553 firmware from the command line with CMake. It
reuses the compiler, core, variant, and MspFlash library that Energia installs,
so the Energia IDE does not need to be open — but Energia (with its MSP430
platform) does need to be installed.


Requirements
------------
- CMake 3.14 or later.
- Energia, installed via its board manager, including:
    - the MSP430 platform  (energia/hardware/msp430/1.0.7)
    - the bundled toolchain (msp430-gcc 4.6.3)
  The default location is:
      ~/Library/Energia15/packages/energia
  If yours differs, pass -DENERGIA_ROOT=<path to .../packages/energia>.
- mspdebug, for flashing:  brew install mspdebug


Build
-----
    cd source/nexus_midi
    cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/msp430_toolchain.cmake
    cmake --build build

Outputs land in build/:
    nexus_midi.elf    the linked firmware
    nexus_midi.hex    Intel HEX, for flashing
A flash/RAM size report is printed at the end of the build.


Flashing
--------
A classic MSP430G2 LaunchPad's onboard FET uses the mspdebug rf2500 driver:

    cmake --build build --target upload

For a newer MSP-EXP430G2ET board (eZ-FET), flash manually with tilib:

    mspdebug tilib --force-reset "prog build/nexus_midi.hex"


How it works
------------
- cmake/msp430_toolchain.cmake points CMake at the Energia-bundled
  msp430-gcc / msp430-g++ and sets -mmcu=msp430g2553.
- CMakeLists.txt mirrors Energia's platform.txt recipe (same flags and
  defines), compiles the Energia core + MspFlash into a static library, and
  links the firmware the way Energia does (with msp430-gcc, -fno-exceptions
  -fno-rtti -Wl,--gc-sections,-u,main).
- The sketch stays a .ino so the Energia IDE still works. The build generates
  build/nexus_midi.cpp from it (prepending <Energia.h>) and compiles that.


Equivalence to the Energia IDE build
------------------------------------
Verified against Energia's own build of the same source: identical program
size (7548 B) and RAM use (296 B), and an identical symbol+size inventory
(95 symbols). Only the link order (absolute addresses) differs.


Targeting another part
----------------------
Defaults to MSP430G2553 / MSP-EXP430G2553LP. For another value-line part pass
-DMSP430_MCU=<part> and use the matching variant headers.


Notes
-----
- A source-level debug build (-g -Og, for msp430-gdb + mspdebug) is not wired
  up yet; the default build matches Energia's release flags (-Os).
- The host-side unit tests were removed in the dead-code cleanup and will be
  re-added against the shipped pitch-bend pipeline.
