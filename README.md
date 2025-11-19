# Flexiv Test Bed

This repository hosts a small collection of Flexiv RDK based experiments that can be run from a single executable.  
The code is organised around a lightweight testing framework (`FlexivRobotTest`) that handles connection lifecycle, logging, and cooperative stop requests.  
Concrete tests (velocity tracking, step sensing, etc.) derive from the base class and implement their own motion routines.

## Repository Layout

```
.
├── CMakeLists.txt
├── conanfile.txt
├── inc/
│   └── flexiv_tests/
│       ├── flexiv_robot_test.h      # Base test interface
│       ├── velocity_tracking_test.h # Velocity tracking scenarios
│       └── step_sensing_test.h      # Step force-sensing routine
├── src/
│   ├── Main.cpp                     # Entry point, ESC monitor, warm starts tests
│   ├── core/
│   │   └── data_logger.cpp          # Timestamped CSV logger
│   └── flexiv_tests/
│       ├── flexiv_robot_test.cpp    # Base test implementation
│       ├── velocity_tracking_test.cpp
│       └── step_sensing_test.cpp
└── logs/                            # Output CSVs (created at runtime)
```

### Test Modules

| File | Purpose |
| ---- | ------- |
| `flexiv_robot_test.*` | Base class providing lifecycle management, logging helpers, thread supervision, stop handling, and shared robot state. |
| `velocity_tracking_test.*` | Implements three velocity tracking routines: static hold (single shot), static loop (repeated stability check), and periodic sine motion (configurable amplitude/period). Each routine reuse the base logging contract to capture target/measured velocities and TCP pose. |
| `step_sensing_test.*` | A reference hybrid motion-force test. Iterates over force/velocity combinations: zeroes the FT sensor, moves to a start pose, seeks surface contact, and applies the target wrench while logging. |
| `core/data_logger.cpp` | Threaded CSV logger with timestamp column. `FlexivRobotTest` configures it with a callback so each concrete test can customise the payload. |
| `src/Main.cpp` | Chooses which test to run (velocity tracking by default), wires up the ESC listener, calls `FlexivRobotTest::runTest()`, and waits for completion. |

## Building with Conan + CMake

Prerequisites:

- Conan 2.x
- CMake ≥3.22
- A compiler supported by Flexiv RDK (MSVC on Windows, GCC/Clang on Linux)

The repository already contains a `conanfile.txt` and CMakeLists, so the usual Conan/CMake workflow works:

```bash
# 1. (optional) Create build directory
cmake -S . -B build

# 2. Install dependencies via Conan (generates toolchains/cache info)
conan install . -s build_type=Release -of build --build=missing

# 3. Configure with the generated toolchain
cmake -S . -B build -G "Ninja" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake

# 4. Build the executable
cmake --build build --config Release
```

On Windows you can substitute the generator (`-G "Ninja"`) with `"Visual Studio 17 2022"` or use `cmake --preset` if you have `CMakeUserPresets.json`.

## Running

After building, run the executable. Example on Windows from the repository root:

```powershell
.\build\Release\Flexiv_Test_Bed.exe
```

The program prints connection progress, starts the configured tests, and listens for `ESC` to stop safely.  
Log files are written to `logs/<TestName>/<phase>.csv` with columns:

