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

#include "targets/tree.h"
#include "testbed.h"

#define DRONE_COUNT 512
#define TREE_COUNT 50000
#define BORDER_WIDTH 2000.0f
#define BORDER_HEIGHT 2000.0f
#define MAX_TIME 1200.0f

static void setupInteractions(swarm::BaseContactListener &listener) {
  listener.addCollisionHandler(
      typeid(swarm::Drone), typeid(swarm::Target),
      [](b2Fixture *droneFixture, b2Fixture *treeFixture) -> void {
        swarm::Target *target = reinterpret_cast<swarm::UserData *>(
                                    treeFixture->GetUserData().pointer)
                                    ->as<swarm::Target>();
        swarm::Drone *drone = reinterpret_cast<swarm::UserData *>(
                                  droneFixture->GetUserData().pointer)
                                  ->as<swarm::Drone>();
      });
  listener.addCollisionHandler(
      typeid(swarm::Drone), typeid(swarm::Drone),
      [](b2Fixture *droneFixture1, b2Fixture *droneFixture2) -> void {
        swarm::Drone *drone1 = reinterpret_cast<swarm::UserData *>(
                                   droneFixture1->GetUserData().pointer)
                                   ->as<swarm::Drone>();
        swarm::Drone *drone2 = reinterpret_cast<swarm::UserData *>(
                                   droneFixture2->GetUserData().pointer)
                                   ->as<swarm::Drone>();
        if (droneFixture1->IsSensor() && droneFixture2->IsSensor()) {
          return;
        }
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
  swarm::CollisionManager::registerType(typeid(swarm::Drone), {typeid(Tree)});
  swarm::CollisionManager::registerType(typeid(Tree), {typeid(swarm::Drone)});
  swarm::TargetFactory::registerTargetType<Tree, b2World *, const b2Vec2 &, int,
                                           bool, bool, float>("Tree");

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

  swarm::TestConfig config = {"Flocking", flock_params, smallDrone, map1,
                              100,        100,          1200.0f,    "Tree"};

  swarm::TestConfig config2 = {"Pheromone Avoidance",
                               pheromone_params,
                               smallDrone,
                               map1,
                               100,
                               200,
                               1200.0f,
                               "Tree"};

  swarm::TestQueue queue;
  config.num_drones = 50;
  queue.push(config);
  queue.push(config2);
  config.num_drones = 50;
  queue.push(config);

  testbed::run();
  return 0;
}