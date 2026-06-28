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
    - the bundled toolchain (msp430-gcc 4.6.6)
  The default location is:
      ~/Library/Energia15/packages/energia
  If yours differs, pass -DENERGIA_ROOT=<path to .../packages/energia>.
- mspdebug, for flashing:  brew install mspdebug
- A host C++ compiler (clang/gcc) for the off-device test harness (below).


Build (device firmware)
-----------------------
    cd source/nexus_midi
    cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/msp430_toolchain.cmake
    cmake --build build

Outputs land in build/:
    nexus_midi.elf    the linked firmware
    nexus_midi.hex    Intel HEX, for flashing
A flash/RAM size report is printed at the end of the build. The current
production image is ~9234 B flash (text 9230 + data 4) and 330 B RAM (bss 326 +
data 4) on the 16 KB / 512 B G2553.

Two build-time options (both default OFF -> the production image):
  -DNEXUS_DIAG=ON   SysEx telemetry for the pitch bend (free-running state +
                    spike snapshots). The BENCH build: flash this to record and
                    diagnose. Adds ~600 B of .text. Keep it in its own dir:
        cmake -B build_diag -DCMAKE_TOOLCHAIN_FILE=cmake/msp430_toolchain.cmake -DNEXUS_DIAG=ON
        cmake --build build_diag
  -DNEXUS_SIM=ON    Route raw_adc() to a synthetic ADC generator on-device (a
                    hardware-in-the-loop self-stimulus). Rarely needed on device;
                    the host harness below is the usual way to run the sim.


Tests (host-side, off device)
-----------------------------
A standalone CMake project under test/ compiles the firmware blocks with the
HOST compiler (do NOT pass the MSP430 toolchain file). Two executables:

    cmake -S test -B test/build
    cmake --build test/build
    ./test/build/unit_blocks     # per-block contracts, each block in isolation
    ./test/build/host_sim        # whole-loop replay of a synthetic gesture suite
    ctest --test-dir test/build  # or run both via CTest

- unit_blocks (NEXUS_HOST): exercises each processor block (slew_gate, stillness,
  dc_servo, output_stage, settle_gate, adc::decimate, ...) in isolation, so a
  failure localizes to one block.
- host_sim (NEXUS_HOST + NEXUS_SIM): runs the full pitch-bend pipeline against a
  deterministic synthetic stimulus and asserts the decoded MIDI stream. It also
  takes --feed <csv> to REPLAY a real captured ADC session (see
  test/replay_captures.py), which is the primary center-stability regression.

Both are the integration + unit gate for firmware changes; run them before
flashing. (After switching git branches use --clean-first for the host build —
CMake can miss a header-only dependency and leave a stale binary.)


Flashing
--------
A classic MSP430G2 LaunchPad's onboard FET uses the mspdebug rf2500 driver. The
convenience target flashes the production image:

    cmake --build build --target upload

To flash any specific image directly (e.g. the diag build) with the same driver:

    mspdebug rf2500 --force-reset "prog build_diag/nexus_midi.hex"

For a newer MSP-EXP430G2ET board (eZ-FET), flash with tilib instead:

    mspdebug tilib --force-reset "prog build/nexus_midi.hex"

If the FET wedges (error code 4 / reply-type mismatch), USB power-cycle the
LaunchPad and retry. The eWhammy target has its own power; unplugging the
programmer (SBW) does not kill it but shifts the ADC reference.


How it works
------------
- cmake/msp430_toolchain.cmake points CMake at the Energia-bundled
  msp430-gcc / msp430-g++ (tools/msp430-gcc/4.6.6) and sets -mmcu=msp430g2553.
- CMakeLists.txt mirrors Energia's platform.txt recipe (same flags and
  defines), compiles the Energia core + MspFlash into a static library, and
  links the firmware the way Energia does (with msp430-gcc, -fno-exceptions
  -fno-rtti -Wl,--gc-sections,-u,main).
- The sketch stays a .ino so the Energia IDE still works. The build generates
  build/nexus_midi.cpp from it (via cmake/gen_sketch.cmake, prepending
  <Energia.h>) and compiles that.


Equivalence to the Energia IDE build
------------------------------------
The CMake build mirrors Energia's platform.txt recipe exactly (flags, defines,
core sources, link), and was verified byte-for-byte against the IDE's own build
of the same source — only the absolute link addresses differ. That check was on
an earlier, smaller image (7548 B); the recipe is unchanged, so the equivalence
holds. See the size report at the end of the build for the current numbers.


Targeting another part
----------------------
Defaults to MSP430G2553 / MSP-EXP430G2553LP. For another value-line part pass
-DMSP430_MCU=<part> and use the matching variant headers.


Notes
-----
- The device toolchain is msp430-gcc 4.6.6, which is C++11-incomplete: no NSDMI
  (use explicit constructors), narrowing brace-init is only a warning, etc. The
  lenient host build can hide errors the device build rejects, so always confirm
  build_diag (the device build) compiles, not just the host harness.
- A source-level debug build (-g -Og, for msp430-gdb) is not wired up yet; the
  default build matches Energia's release flags (-Os). You can still debug the
  release image at runtime via mspdebug (setbreak / regs / md).
