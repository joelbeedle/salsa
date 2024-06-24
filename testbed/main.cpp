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

int main() {
  auto map1 = swarm::map::load("new_map");
  auto map2 = swarm::map::load("test2");

  auto flock_params = swarm::behaviour::Registry::getInstance()
                          .getBehaviour("Flocking")
                          ->getParameters();

  swarm::DroneConfiguration *smallDrone = new swarm::DroneConfiguration(
      25.0f, 50.0f, 10.0f, 0.3f, 1.0f, 1.5f, 4000.0f);
  swarm::CollisionManager::registerType<swarm::Drone>({typeid(Tree).name()});
  swarm::CollisionManager::registerType<Tree>({typeid(swarm::Drone).name()});
  swarm::TargetFactory::registerTargetType<Tree, bool, bool, float>("Tree");

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

  auto contactListener = std::make_shared<swarm::BaseContactListener>();
  setupInteractions(*contactListener);

  swarm::TestConfig config = {"Flocking",
                              flock_params,
                              smallDrone,
                              map1,
                              100,
                              100,
                              1200.0f,
                              "Tree",
                              std::tuple(false, false, 5.0f),
                              contactListener.get()};

  swarm::TestQueue queue;
  config.num_drones = 100;
  config.num_targets = 0;
  queue.push(config);
  config.behaviour_name = "Pheromone Avoidance";
  queue.push(config);
  config.num_drones = 50;
  queue.push(config);

  testbed::run();
  return 0;
}