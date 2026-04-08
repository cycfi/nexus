Building and Running the Tests
===============================

Requirements
------------
- CMake 3.14 or later
- A C++11-capable compiler (GCC, Clang, MSVC)

Steps
-----
    cd source/nexus_midi
    cmake -B build
    cmake --build build
    ctest --test-dir build --output-on-failure

The test binary can also be run directly for the full output including
the step response table:

    ./build/test/test_dynamic_smoother        (macOS / Linux)
    build\test\test_dynamic_smoother.exe      (Windows)
