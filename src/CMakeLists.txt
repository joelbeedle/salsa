file(GLOB_RECURSE HEADER_LIST CONFIGURE_DEPENDS "${SwarmSim_SOURCE_DIR}/include/*.h")
file(GLOB_RECURSE SOURCE_LIST CONFIGURE_DEPENDS "${SwarmSim_SOURCE_DIR}/src/*.cpp")

#if(NOT TARGET spdlog)
#    find_package(spdlog REQUIRED)
#endif()

FetchContent_Declare(
  spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG        v1.14.1  # Optionally pin to a specific tag or branch
)

# Make spdlog available
FetchContent_MakeAvailable(spdlog)

add_library(swarm-sim ${SOURCE_LIST} ${HEADER_LIST})

target_link_libraries(swarm-sim PRIVATE spdlog::spdlog box2d)
target_include_directories(swarm-sim PUBLIC ${FREETYPE_INCLUDE_DIRS} ../include)

target_compile_features(swarm-sim PUBLIC cxx_std_17)

source_group(
    TREE "${PROJECT_SOURCE_DIR}/include"
    PREFIX "Header Files"
    FILES ${HEADER_LIST}
)