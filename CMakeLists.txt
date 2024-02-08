cmake_minimum_required(VERSION 3.5)
project(swarm-box2d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(BUILD_TESTBED "Build the Box2D testbed" ON)

# Add SFML submodule
add_subdirectory(lib/SFML)
include_directories(lib/SFML/include)

# Add Box2D submodule
add_subdirectory(lib/Box2D)
include_directories(lib/Box2D)

# Source files
set(SOURCES
    src/main.cpp
    src/Drone.cpp
    src/FlockingBehaviour.cpp
    src/PheremoneBehaviour.cpp
    src/PSOBehaviour.cpp
    src/Tree.cpp)

# Executable
add_executable(swarm-box2d ${SOURCES})

# Linking
target_link_libraries(swarm-box2d PRIVATE box2d)

target_link_libraries(swarm-box2d PRIVATE sfml-system sfml-network sfml-graphics sfml-window)


# Include directory
target_include_directories(swarm-box2d PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_compile_features(swarm-box2d PRIVATE cxx_std_17)

# Add TestBed
if(BUILD_TESTBED)
    add_subdirectory(testbed)

    if (MSVC)
        set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT testbed)
        set_property(TARGET testbed PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/testbed")
    endif()

endif()