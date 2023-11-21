cmake_minimum_required(VERSION 3.5)
project(swarm-box2d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)


# Add SFML submodule
add_subdirectory(lib/SFML)
include_directories(lib/SFML/include)

# Add Box2D submodule
add_subdirectory(lib/Box2D)
include_directories(lib/Box2D)

# External Packages
#find_package(box2D CONFIG REQUIRED)

#find_package(SFML COMPONENTS system window graphics REQUIRED)

# Include directories
#include_directories(${Box2D_INCLUDE_DIR})

# Source files
set(SOURCES src/main.cpp)

# Executable
add_executable(swarm-box2d ${SOURCES})

# Linking
target_link_libraries(swarm-box2d PRIVATE box2d)

target_link_libraries(swarm-box2d PRIVATE sfml-system sfml-network sfml-graphics sfml-window)


# Include directory
target_include_directories(swarm-box2d PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_compile_features(swarm-box2d PRIVATE cxx_std_17)

