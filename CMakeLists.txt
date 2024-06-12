cmake_minimum_required(VERSION 3.5)
project(swarm-sim VERSION 1.0 DESCRIPTION "Drone Swarm Simulation System")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(SS_BUILD_TESTBED "Build the Box2D testbed" ON)
option(SS_BUILD_DOCS "Build the documentation" ON)
add_compile_definitions(_USE_MATH_DEFINES)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Add Box2D submodule, and include it
add_subdirectory(extern/Box2D)
# include_directories(extern/Box2D)


add_subdirectory(src)

# Compile sources into a library
# Add header files to the library. Set to PUBLIC so users of the library can see these
target_include_directories(libswarm-sim PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_compile_features(libswarm-sim PUBLIC cxx_std_17)

if(SS_BUILD_DOCS)
    add_subdirectory(docs)
endif()

enable_testing()
add_subdirectory(test)

# Handle building TestBed, if that's what we want
if(SS_BUILD_TESTBED)
    add_subdirectory(testbed)
    if (MSVC)
        set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT testbed)
        set_property(TARGET testbed PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/testbed")
    endif()
endif()