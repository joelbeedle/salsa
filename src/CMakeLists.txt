file(GLOB_RECURSE HEADER_LIST CONFIGURE_DEPENDS "${SwarmSim_SOURCE_DIR}/include/*.h")
file(GLOB_RECURSE SOURCE_LIST CONFIGURE_DEPENDS "${SwarmSim_SOURCE_DIR}/src/*.cpp")


add_library(swarm-sim ${SOURCE_LIST} ${HEADER_LIST})

target_link_libraries(swarm-sim PRIVATE box2d)
target_include_directories(swarm-sim PUBLIC ${FREETYPE_INCLUDE_DIRS} ../include)

target_compile_features(swarm-sim PUBLIC cxx_std_17)

source_group(
    TREE "${PROJECT_SOURCE_DIR}/include"
    PREFIX "Header Files"
    FILES ${HEADER_LIST}
)