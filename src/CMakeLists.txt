file(GLOB_RECURSE HEADER_LIST CONFIGURE_DEPENDS "${SwarmSim_SOURCE_DIR}/include/*.h")
file(GLOB_RECURSE SOURCE_LIST CONFIGURE_DEPENDS "${SwarmSim_SOURCE_DIR}/src/*.cpp")


FetchContent_Declare(
  spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG        v1.14.1
)
FetchContent_Declare(
	nlohmann_json
	GIT_REPOSITORY https://github.com/nlohmann/json.git
	GIT_TAG        v3.11.3)

FetchContent_MakeAvailable(nlohmann_json)
# Make spdlog available
FetchContent_MakeAvailable(spdlog)

add_library(salsa ${SOURCE_LIST} ${HEADER_LIST})

target_link_libraries(salsa PRIVATE nlohmann_json::nlohmann_json spdlog::spdlog box2d)
target_include_directories(salsa PRIVATE ${spdlog_SOURCE_DIR}/include ${nlohmann_json_SOURCE_DIR}/include ${FREETYPE_INCLUDE_DIRS})
target_include_directories(salsa PUBLIC ../include)

target_compile_features(salsa PUBLIC cxx_std_17)

source_group(
    TREE "${PROJECT_SOURCE_DIR}/include"
    PREFIX "Header Files"
    FILES ${HEADER_LIST}
)