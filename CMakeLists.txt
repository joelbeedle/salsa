cmake_minimum_required(VERSION 3.5)
project(swarm-box2d)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(box2D CONFIG REQUIRED)

find_package(SFML COMPONENTS system window graphics REQUIRED)
# Include directories
include_directories(${Box2D_INCLUDE_DIR})

#include_directories(${SFML_INCLUDE_DIRS})

# If you want SFML to provide an implementation of main():
#target_link_libraries(main PRIVATE sfml-main)
# Source files
set(SOURCE_FILES main.cpp)

# Executable
add_executable(swarm-box2d ${SOURCE_FILES})

# Linking
target_link_libraries(swarm-box2d PRIVATE box2d::box2d)

target_link_libraries(swarm-box2d PRIVATE sfml-system sfml-network sfml-graphics sfml-window)
