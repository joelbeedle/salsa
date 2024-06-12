cmake_policy(SET CMP0079 NEW)

set (MY_TESTBED_SOURCE_FILES
	draw.cpp
	draw.h
	imgui_impl_glfw.cpp
	imgui_impl_glfw.h
	imgui_impl_opengl3.cpp
	imgui_impl_opengl3.h
	run_sim.cpp
	run_sim.h
	settings.h
	settings.cpp
	test.cpp
	test.h
	swarm.cpp
	swarm.h
	main.cpp
	${PROJECT_SOURCE_DIR}/testbed/behaviours/pheromone_avoidance.cpp
${PROJECT_SOURCE_DIR}/testbed/behaviours/flocking.cpp
${PROJECT_SOURCE_DIR}/testbed/behaviours/levy_flocking.cpp
${PROJECT_SOURCE_DIR}/testbed/behaviours/flocking.cpp
${PROJECT_SOURCE_DIR}/testbed/behaviours/uniform_random_walk.cpp
${PROJECT_SOURCE_DIR}/testbed/behaviours/dsp.cpp
	# tests/drone_swarm_test.cpp
	# tests/swarm_test.cpp
)
set(MY_PROJECT_SOURCE_FILES

)
set(MY_PROJECT_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../include")

add_executable(my_testbed ${MY_PROJECT_SOURCE_FILES} ${MY_TESTBED_SOURCE_FILES} )
target_sources(my_testbed PRIVATE ${PROJECT_SOURCES})
target_include_directories(my_testbed PRIVATE
${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(my_testbed PRIVATE glfw imgui sajson glad box2d libswarm-sim)

set_target_properties(my_testbed PROPERTIES
	CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# message(STATUS "runtime = ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
# message(STATUS "binary = ${CMAKE_CURRENT_BINARY_DIR}")

# Copy font files, etc


#source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${MY_TESTBED_SOURCE_FILES})


