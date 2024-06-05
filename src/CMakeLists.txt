project(swarm-box2d)

include_directories(${PROJECT_SRC_DIR}extern/Box2D)
include_directories(${PROJECT_SRC_DIR}/include)

add_library(swarm-box2d STATIC
    behaviours/behaviour.cpp
    behaviours/dsp.cpp
    behaviours/flocking.cpp
    behaviours/levy_flocking.cpp
    behaviours/pheromone_avoidance.cpp
    behaviours/uniform_random_walk.cpp
    drones/drone.cpp
    tree.cpp
)

target_include_directories(swarm-box2d PUBLIC ${PROJECT_SRC_DIR}/include)

target_link_libraries(swarm-box2d PUBLIC box2d)