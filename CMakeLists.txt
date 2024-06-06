cmake_minimum_required(VERSION 3.5)
project(swarm-sim VERSION 1.0 DESCRIPTION "Drone Swarm Simulation System")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(BUILD_TESTBED "Build the Box2D testbed" ON)
add_compile_definitions(_USE_MATH_DEFINES)

# Add Box2D submodule, and include it
add_subdirectory(extern/Box2D)
# include_directories(extern/Box2D)

add_subdirectory(src)

# --------------------------------------------------------------------------------
#                            Build Configuration
# --------------------------------------------------------------------------------

# Compile sources into a library
# Add header files to the library. Set to PUBLIC so users of the library can see these
target_include_directories(libswarm-sim PUBLIC ${PROJECT_SOURCE_DIR}/include)

# Set the C++ standard to C++17
target_compile_features(libswarm-sim PUBLIC cxx_std_17)

# Handle building TestBed, if that's what we want
if(BUILD_TESTBED)
    add_subdirectory(testbed)
    if (MSVC)
        set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT testbed)
        set_property(TARGET testbed PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/testbed")
    endif()
endif()