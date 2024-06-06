#include <box2d/box2d.h>
#include <stdio.h>
#include <time.h>

#include <algorithm>
#include <chrono>
#include <filesystem>  // C++17 header for directory operations
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <stack>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include "behaviours/dsp.h"
#include "behaviours/flocking.h"
#include "behaviours/levy_flocking.h"
#include "behaviours/pheromone_avoidance.h"
#include "behaviours/registry.h"
#include "behaviours/uniform_random_walk.h"
#include "core/sim.h"
#include "draw.h"
#include "drones/drone.h"
#include "drones/drone_factory.h"
#include "imgui/imgui.h"
#include "settings.h"
#include "swarm.h"
#include "test.h"
#include "tree.h"
#include "utils/drone_configuration.h"
#include "utils/drone_contact_listener.h"
#include "utils/object_types.h"

#define DRONE_COUNT 50
#define TREE_COUNT 50000
#define BORDER_WIDTH 2000.0f
#define BORDER_HEIGHT 2000.0f
#define MAX_TIME 1200.0f

int main() {
  swarm_sim::FlockingParameters flockingParams = {250.0f, 1.6f, 1.0f, 3.0f,
                                                  3.0f};
  auto flock = std::make_unique<swarm_sim::FlockingBehaviour>(flockingParams);
  SwarmTest::AddBehaviour("Flocking", std::move(flock));
  SwarmTest::Run();
  return 0;
}