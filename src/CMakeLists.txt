project(swarm-sim)

include_directories(${PROJECT_SRC_DIR}extern/Box2D)
include_directories(${PROJECT_SRC_DIR}/include)

add_library(libswarm-sim STATIC
    behaviours/behaviour.cpp
    # behaviours/dsp.cpp
    # behaviours/flocking.cpp
    # behaviours/levy_flocking.cpp
    # behaviours/pheromone_avoidance.cpp
    # behaviours/uniform_random_walk.cpp
    drones/drone.cpp
    core/sim.cpp
    tree.cpp
)
target_include_directories(libswarm-sim PUBLIC ${PROJECT_SRC_DIR}/include)
target_sources(libswarm-sim INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include/core/sim.h)
target_link_libraries(libswarm-sim PUBLIC box2d)