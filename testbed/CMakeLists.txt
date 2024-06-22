# cmake_policy(SET CMP0079 NEW)

set (MY_TESTBED_SOURCE_FILES
	draw.cpp
	draw.h
	run_sim.cpp
	run_sim.h
	settings.h
	settings.cpp
	test.cpp
	test.h
	map.cpp
	queue_mode.cpp
	sandbox_mode.cpp
	main.cpp
	behaviours/pheromone_avoidance.cpp
	behaviours/flocking.cpp
	behaviours/uniform_random_walk.cpp
	behaviours/dsp.cpp
	# tests/drone_swarm_test.cpp
	# tests/swarm_test.cpp
)

FetchContent_Declare(
	nlohmann_json
	GIT_REPOSITORY https://github.com/nlohmann/json.git
	GIT_TAG        v3.11.3)
FetchContent_MakeAvailable(nlohmann_json)

add_executable(my_testbed ${MY_TESTBED_SOURCE_FILES})
target_compile_features(my_testbed PRIVATE cxx_std_20)
target_link_libraries(my_testbed PRIVATE glad glfw imgui box2d nlohmann_json::nlohmann_json swarm-sim)

# message(STATUS "runtime = ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
# message(STATUS "binary = ${CMAKE_CURRENT_BINARY_DIR}")

# Copy font files, etc

#source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${MY_TESTBED_SOURCE_FILES})


