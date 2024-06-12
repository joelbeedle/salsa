# cmake_policy(SET CMP0079 NEW)

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
	behaviours/pheromone_avoidance.h
	behaviours/pheromone_avoidance.cpp
	behaviours/flocking.cpp
	behaviours/levy_flocking.cpp
	behaviours/flocking.cpp
	behaviours/uniform_random_walk.cpp
	behaviours/dsp.cpp
	# tests/drone_swarm_test.cpp
	# tests/swarm_test.cpp
)

find_package(Freetype REQUIRED)

add_executable(my_testbed ${MY_TESTBED_SOURCE_FILES} )
target_compile_features(my_testbed PRIVATE cxx_std_20)
target_link_libraries(my_testbed PRIVATE glfw imgui sajson glad box2d ${FREETYPE_LIBRARIES} swarm-sim)

# message(STATUS "runtime = ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
# message(STATUS "binary = ${CMAKE_CURRENT_BINARY_DIR}")

# Copy font files, etc

#source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${MY_TESTBED_SOURCE_FILES})


