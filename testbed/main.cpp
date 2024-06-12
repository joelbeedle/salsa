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
#include "core/test_stack.h"
#include "draw.h"
#include "drones/drone.h"
#include "drones/drone_factory.h"
#include "imgui/imgui.h"
#include "map.h"
#include "settings.h"
#include "swarm.h"
#include "test.h"
#include "utils/collision_manager.h"
#include "utils/drone_configuration.h"
#include "utils/object_types.h"
#include "utils/tree.h"

#define DRONE_COUNT 512
#define TREE_COUNT 50000
#define BORDER_WIDTH 2000.0f
#define BORDER_HEIGHT 2000.0f
#define MAX_TIME 1200.0f

static void setupInteractions(swarm::BaseContactListener &listener) {
  listener.addCollisionHandler(
      typeid(swarm::Drone), typeid(swarm::Tree),
      [](b2Fixture *droneFixture, b2Fixture *treeFixture) -> void {
        swarm::Tree *tree = reinterpret_cast<swarm::UserData *>(
                                treeFixture->GetUserData().pointer)
                                ->as<swarm::Tree>();
        swarm::Drone *drone = reinterpret_cast<swarm::UserData *>(
                                  droneFixture->GetUserData().pointer)
                                  ->as<swarm::Drone>();

        if (drone && tree) {
          tree->setMapped(true);
          tree->addNumMapped();
        }
      });
}

int main() {
  auto flock =
      std::make_unique<swarm::FlockingBehaviour>(250.0, 1.6, 1.0, 3.0, 3.0);
  auto flock_params = flock.get()->getParameters();

  auto pheromone = std::make_unique<swarm::PheromoneBehaviour>(0.5, 1.0);
  swarm::DroneConfiguration *smallDrone = new swarm::DroneConfiguration(
      25.0f, 50.0f, 10.0f, 0.3f, 1.0f, 1.5f, 4000.0f);
  swarm::CollisionManager::registerType(typeid(swarm::Drone),
                                        {typeid(swarm::Tree)});
  swarm::CollisionManager::registerType(typeid(swarm::Tree),
                                        {typeid(swarm::Drone)});
  swarm::behaviour::Registry::getInstance().add("Flocking", std::move(flock));
  swarm::behaviour::Registry::getInstance().add("Pheromone Avoidance",
                                                std::move(pheromone));

  auto new_flock_params = std::unordered_map<std::string, float>({
      {"Separation Distance", 300.0},
      {"Alignment Weight", 1.6},
      {"Cohesion Weight", 1.0},
      {"Separation Weight", 1.0},
      {"Obstacle Avoidance Weight", 1.0},
  });

  auto pheromone_params = std::unordered_map<std::string, float>({
      {"Decay Rate", 0.5},
      {"Obstacle Avoidance Weight", 1.0},
  });

  swarm::TestConfig config = {
      "Flocking", new_flock_params, smallDrone, BORDER_HEIGHT, BORDER_WIDTH, 1,
      0,          1200.0f,
  };

  swarm::TestConfig config2 = {
      "Pheromone Avoidance",
      pheromone_params,
      smallDrone,
      500.0f,
      500.0f,
      100,
      0,
      1200.0f,
  };

  swarm::TestStack stack;
  stack.push(config);
  stack.push(config2);
  config.num_drones = 2048;
  stack.push(config);

  std::unique_ptr<SwarmTest> test = std::make_unique<SwarmTest>();
  test->UseStack(stack);
  test->SetStackSim();

  auto contactListener = std::make_shared<swarm::BaseContactListener>();
  setupInteractions(*contactListener);
  test->SetContactListener(*contactListener);
  test->Build();
  RegisterTest("SwarmTest", "Swarm_Test", std::move(test));
  RegisterTest("MapCreator", "Map_Creator", std::make_unique<MapCreator>());
  run_sim();
  return 0;
}