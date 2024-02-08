cmake_policy(SET CMP0079 NEW)

set (MY_TESTBED_SOURCE_FILES
	draw.cpp
	draw.h
	imgui_impl_glfw.cpp
	imgui_impl_glfw.h
	imgui_impl_opengl3.cpp
	imgui_impl_opengl3.h
	main.cpp
	settings.h
	settings.cpp
	test.cpp
	test.h
	tests/DroneSwarmTest.cpp
)
set(MY_PROJECT_SOURCE_FILES
${PROJECT_SOURCE_DIR}/src/Drone.cpp
${PROJECT_SOURCE_DIR}/src/FlockingBehaviour.cpp
${PROJECT_SOURCE_DIR}/src/PheremoneBehaviour.cpp
${PROJECT_SOURCE_DIR}/src/PSOBehaviour.cpp
${PROJECT_SOURCE_DIR}/src/Tree.cpp)
set(MY_PROJECT_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../include")

add_executable(my_testbed ${MY_TESTBED_SOURCE_FILES} ${MY_PROJECT_SOURCE_FILES})
target_sources(my_testbed PRIVATE ${PROJECT_SOURCES})
target_include_directories(my_testbed PRIVATE
${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(my_testbed PRIVATE glfw imgui sajson glad box2d)

set_target_properties(my_testbed PROPERTIES
	CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# message(STATUS "runtime = ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
# message(STATUS "binary = ${CMAKE_CURRENT_BINARY_DIR}")

# Copy font files, etc


#source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${MY_TESTBED_SOURCE_FILES})


