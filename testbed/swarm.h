#ifndef SWARM_H
#define SWARM_H
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
#include "draw.h"
#include "drones/drone.h"
#include "drones/drone_factory.h"
#include "imgui/imgui.h"
#include "run_sim.h"
#include "settings.h"
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
struct DroneParameters {
  float cameraViewRange;
  float obstacleViewRange;
  float droneDetectionRange;
  float maxSpeed;
  float maxForce;
  float mass;
  float radius;
};
class SwarmTest : public Test {
 private:
  swarm_sim::Behaviour* behaviour_;
  std::string current_behaviour_name_;
  swarm_sim::DroneContactListener contact_listener_;

  // Setup drone parameters and configurations
  std::unordered_map<std::string, DroneParameters> all_drone_parameters_;
  std::unordered_map<std::string, swarm_sim::DroneConfiguration>
      all_drone_configurations_;

  swarm_sim::DroneConfiguration* drone_configuration_;
  DroneParameters* drone_parameters_;

  std::vector<std::unique_ptr<swarm_sim::Drone>> drones_;

  float obstacle_view_range_;
  float camera_view_range_;
  float max_speed_;
  float max_force_;

  float num_drones_;

 public:
  void initWorld() {
    g_debugDraw.SetFlags(b2Draw::e_shapeBit | b2Draw::e_jointBit);
    b2Vec2 gravity(0.0f, 0.0f);
    m_world->SetGravity(gravity);
  }

  static void AddBehaviour(const std::string& name,
                           std::unique_ptr<swarm_sim::Behaviour> behaviour) {
    swarm_sim::behaviour::Registry::getInstance().add(name,
                                                      std::move(behaviour));
  }
  static Test* Create() {
    // Generate a new version of this test with the user defined settings
    return new SwarmTest;
  }
  void Step(Settings& settings) override {
    // Run simulation steps here
    Test::Step(settings);
  }
  static void Run() {
    RegisterTest("SwarmTest", "Swarm_Test", SwarmTest::Create);
    run_sim();
  }
  // Accessors and Mutators

  swarm_sim::DroneConfiguration* drone_configuration() {
    return drone_configuration_;
  }
  const swarm_sim::DroneConfiguration* drone_configuration() const {
    return drone_configuration_;
  }

  DroneParameters* drone_parameters() { return drone_parameters_; }
  const DroneParameters* drone_parameters() const { return drone_parameters_; }
};
#endif  // SWARM_H