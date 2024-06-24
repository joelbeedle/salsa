# cmake_policy(SET CMP0079 NEW)

set (MY_TESTBED_SOURCE_FILES
	draw.cpp
	draw.h
	testbed.cpp
	testbed.h
	settings.h
	settings.cpp
	test.cpp
	test.h
	user.cpp
	user.h
	plot/plot.cpp
	plot/plot.h
	targets/tree.cpp
	targets/tree.h
	modes/map_creator.cpp
	modes/queue_mode.cpp
	modes/sandbox_mode.cpp
	behaviours/pheromone_avoidance.cpp
	behaviours/flocking.cpp
	behaviours/uniform_random_walk.cpp
	behaviours/dsp.cpp
	main.cpp
	# tests/drone_swarm_test.cpp
	# tests/swarm_test.cpp
)

FetchContent_Declare(
	nlohmann_json
	GIT_REPOSITORY https://github.com/nlohmann/json.git
	GIT_TAG        v3.11.3)

FetchContent_Declare(
	cli11_proj
	QUIET
	GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
	GIT_TAG 6c7b07a878ad834957b98d0f9ce1dbe0cb204fc9)

FetchContent_MakeAvailable(cli11_proj)

FetchContent_MakeAvailable(nlohmann_json)

find_package (Python REQUIRED
        COMPONENTS Interpreter Development)

add_executable(testbed ${MY_TESTBED_SOURCE_FILES})
target_include_directories(testbed PRIVATE  ${Python_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(testbed PRIVATE cxx_std_20)
target_link_libraries(testbed PRIVATE glad glfw imgui box2d spdlog::spdlog nlohmann_json::nlohmann_json CLI11::CLI11 ${Python_LIBRARIES} swarm-sim)

# message(STATUS "runtime = ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
# message(STATUS "binary = ${CMAKE_CURRENT_BINARY_DIR}")

# Copy font files, etc

#source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${MY_TESTBED_SOURCE_FILES})


