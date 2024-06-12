cmake_minimum_required(VERSION 3.14...3.51)

project(SwarmSim
 VERSION 1.0
 DESCRIPTION "Drone Swarm Simulation System"
 LANGUAGES CXX)

 if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)

    set(CMAKE_CXX_EXTENSIONS OFF)
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)


    find_package(Doxygen)
    if(Doxygen_FOUND)
        add_subdirectory(docs)
    else()
        message(STATUS "Doxygen not found, not building documentation") 
    endif()
endif()

include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_compile_definitions(_USE_MATH_DEFINES)


# Add Box2D submodule, and include it
add_subdirectory(extern/Box2D)

# Compiled Library code is here
add_subdirectory(src)

# Testbed code here
add_subdirectory(testbed)

if((CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME OR MODERN_CMAKE_BUILD_TESTING)
   AND BUILD_TESTING)
   enable_testing()
   add_subdirectory(test)
endif()

