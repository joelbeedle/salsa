#include "user.h"

#include <box2d/box2d.h>
#include <stdio.h>
#include <time.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
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

static void setupInteractions(salsa::BaseContactListener &listener) {
  listener.addCollisionHandler(
      "salsa::Drone", "Tree",
      [](b2Fixture *droneFixture, b2Fixture *treeFixture) -> void {
        salsa::Drone *drone = reinterpret_cast<salsa::UserData *>(
                                  droneFixture->GetUserData().pointer)
                                  ->as<salsa::Drone>();
        Tree *tree = reinterpret_cast<salsa::UserData *>(
                         treeFixture->GetUserData().pointer)
                         ->as<Tree>();
        tree->setFound(true);
        drone->addTargetFound(tree);
        tree->addNumMapped();
      });
  listener.addCollisionHandler(
      "salsa::Drone", "salsa::Drone",
      [](b2Fixture *droneFixture1, b2Fixture *droneFixture2) -> void {

      });
}

void user() {
  auto flock_params =
      salsa::behaviour::Registry::get().behaviour("Flocking")->getParameters();

  salsa::DroneConfiguration *smallDrone = new salsa::DroneConfiguration(
      "Small", 15.0f, 50.2f, 10.0f, 0.3f, 1.0f, 1.5f, 134.0f);
  salsa::CollisionManager::registerType<salsa::Drone>({typeid(Tree).name()});
  salsa::CollisionManager::registerType<Tree>({typeid(salsa::Drone).name()});
  salsa::TargetFactory::registerTarget<Tree, bool, bool, float>("Tree", false,
                                                                false, 5.0f);

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

  static auto contactListener =
      std::make_shared<salsa::BaseContactListener>("Default");
  setupInteractions(*contactListener);
  salsa::TestConfig config = {"Flocking", flock_params, "Small", "speed",  100,
                              100,        100.0f,       "Tree",  "Default"};
  json j = config;                      // Serialize to JSON
  std::cout << j.dump(4) << std::endl;  // Print JSON

  std::vector<std::vector<float>> loaded_permutations;
  std::vector<std::string> loaded_parameter_names;
  // salsa::loadPermutations(loaded_permutations, loaded_parameter_names,
  //                         "permutations.json");
  // queue.addPermutedTests(config, loaded_permutations,
  // loaded_parameter_names);
  salsa::TestQueue::load("permutations");
}

}  // namespace testbed