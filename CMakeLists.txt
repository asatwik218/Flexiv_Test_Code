cmake_minimum_required(VERSION 3.16.3)

project(Flexiv_Test_Bed VERSION 1.0)

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()
# Export compile_commands.json for IDEs (VSCode IntelliSense)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Source and include directories
set(SRC_DIR ${CMAKE_SOURCE_DIR}/src)
set(INC_DIR ${CMAKE_SOURCE_DIR}/inc)
include_directories(${INC_DIR})


# Tell CMake where to find packages (this affects find_package)
if(FLEXIV_RDK_DIR_HINT)
  list(APPEND CMAKE_PREFIX_PATH ${FLEXIV_RDK_DIR_HINT})
endif()

# Find Flexiv RDK package
find_package(flexiv_rdk REQUIRED)
find_package(Eigen3 REQUIRED)

# ----------------------------------------------------------
# Gather source files
# ----------------------------------------------------------
# (Glob everything in src/core and src/flexiv)
file(GLOB CORE_SOURCES "${SRC_DIR}/core/*.cpp")
file(GLOB FLEXIV_SOURCES "${SRC_DIR}/flexiv_tests/*.cpp")

# Combine everything for one build target
set(ALL_SOURCES
  ${CORE_SOURCES}
  ${FLEXIV_SOURCES}
  ${SRC_DIR}/Main.cpp
)

# ----------------------------------------------------------
# Build the main executable
# ----------------------------------------------------------
add_executable(Flexiv_Test_Bed ${ALL_SOURCES})

target_link_libraries( Flexiv_Test_Bed
    PRIVATE
        flexiv::rdk          # public headers depend on flexiv
        Eigen3::Eigen 
)

# Link Windows system library only when building on Windows
if(WIN32)
    target_link_libraries(Flexiv_Test_Bed PRIVATE winmm)
    # Set stack reserve to 8 MiB
    target_link_options(Flexiv_Test_Bed PRIVATE "/STACK:8388608")
endif()

# ----------------------------------------------------------
# Compiler settings
# ----------------------------------------------------------
set_target_properties(Flexiv_Test_Bed PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)
