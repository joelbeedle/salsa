#include "user.h"

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

#include "spdlog/sinks/stdout_color_sinks.h"
#include "targets/tree.h"
#include "testbed.h"

namespace testbed {

#define DRONE_COUNT 512
#define TREE_COUNT 50000
#define BORDER_WIDTH 2000.0f
#define BORDER_HEIGHT 2000.0f
#define MAX_TIME 1200.0f

static void setupInteractions(swarm::BaseContactListener &listener) {
  listener.addCollisionHandler(
      "swarm::Drone", "Tree",
      [](b2Fixture *droneFixture, b2Fixture *treeFixture) -> void {
        swarm::Drone *drone = reinterpret_cast<swarm::UserData *>(
                                  droneFixture->GetUserData().pointer)
                                  ->as<swarm::Drone>();
        Tree *tree = reinterpret_cast<swarm::UserData *>(
                         treeFixture->GetUserData().pointer)
                         ->as<Tree>();
        tree->setFound(true);
        drone->addTargetFound(tree);
        tree->addNumMapped();
      });
  listener.addCollisionHandler(
      "swarm::Drone", "swarm::Drone",
      [](b2Fixture *droneFixture1, b2Fixture *droneFixture2) -> void {
        swarm::Drone *drone1 = reinterpret_cast<swarm::UserData *>(
                                   droneFixture1->GetUserData().pointer)
                                   ->as<swarm::Drone>();
        swarm::Drone *drone2 = reinterpret_cast<swarm::UserData *>(
                                   droneFixture2->GetUserData().pointer)
                                   ->as<swarm::Drone>();
      });
}

void user() {
  auto flock_params = swarm::behaviour::Registry::getInstance()
                          .getBehaviour("Flocking")
                          ->getParameters();

  swarm::DroneConfiguration *smallDrone = new swarm::DroneConfiguration(
      15.0f, 50.2f, 10.0f, 0.3f, 1.0f, 1.5f, 134.0f);
  swarm::CollisionManager::registerType<swarm::Drone>({typeid(Tree).name()});
  swarm::CollisionManager::registerType<Tree>({typeid(swarm::Drone).name()});
  swarm::TargetFactory::registerTargetType<Tree, bool, bool, float>("Tree");

  auto new_flock_params = std::unordered_map<std::string, float>({
      {"Separation Distance", 173.0},
      {"Alignment Weight", 1.4},
      {"Cohesion Weight", 0.69},
      {"Separation Weight", 4.76},
      {"Obstacle Avoidance Weight", 4.00},
  });

  auto pheromone_params = std::unordered_map<std::string, float>({
      {"Decay Rate", 0.5},
      {"Obstacle Avoidance Weight", 1.0},
  });

  static auto contactListener = std::make_shared<swarm::BaseContactListener>();
  setupInteractions(*contactListener);
  swarm::map::Map map = swarm::map::load("poly");
  swarm::map::Map map1 = {"Map1", BORDER_WIDTH, BORDER_HEIGHT, b2Vec2(0, 0),
                          new b2World(b2Vec2(0.0f, 0.0f))};
  swarm::TestConfig config = {"Flocking",
                              flock_params,
                              smallDrone,
                              map,
                              100,
                              100,
                              1200.0f,
                              "Tree",
                              std::tuple(false, false, 5.0f),
                              contactListener.get()};
  std::vector<std::vector<float>> loaded_permutations;
  std::vector<std::string> loaded_parameter_names;
  // swarm::loadPermutations(loaded_permutations, loaded_parameter_names,
  //                         "permutations.json");
  // queue.addPermutedTests(config, loaded_permutations,
  // loaded_parameter_names);
  config.num_targets = 500;
  config.num_drones = 100;
  config.time_limit = 100.0f;
  swarm::TestQueue::push(config);
}

}  // namespace testbed