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
#include "settings.h"
#include "swarm.h"
#include "test.h"
#include "utils/collision_manager.h"
#include "utils/drone_configuration.h"
#include "utils/object_types.h"
#include "utils/tree.h"

#define DRONE_COUNT 50
#define TREE_COUNT 50000
#define BORDER_WIDTH 2000.0f
#define BORDER_HEIGHT 2000.0f
#define MAX_TIME 1200.0f

static void setupInteractions(swarm::BaseContactListener &listener) {
  listener.addCollisionHandler(
      typeid(swarm::Drone), typeid(swarm::Tree),
      [](b2Fixture *droneFixture, b2Fixture *treeFixture) {
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
  // TODO: Register each behaviour with the registry, when being made, this
  // removes the need for header files
  auto flock =
      std::make_unique<swarm::FlockingBehaviour>(250.0, 1.6, 1.0, 3.0, 3.0);
  auto pheromone = std::make_unique<swarm::PheromoneBehaviour>(0.5, 1.0);
  swarm::DroneConfiguration *smallDrone = new swarm::DroneConfiguration(
      25.0f, 50.0f, 10.0f, 0.3f, 1.0f, 1.5f, 4000.0f);
  SwarmTest::SetHeight(BORDER_HEIGHT);
  SwarmTest::SetWidth(BORDER_WIDTH);
  swarm::CollisionManager::registerType(typeid(swarm::Drone),
                                        {typeid(swarm::Tree)});
  swarm::CollisionManager::registerType(typeid(swarm::Tree),
                                        {typeid(swarm::Drone)});
  swarm::CollisionConfig config =
      swarm::CollisionManager::getCollisionConfig(typeid(swarm::Drone));
  std::bitset<16> catBits(config.categoryBits);
  std::cout << catBits << std::endl;
  std::bitset<16> maskBits(config.maskBits);
  std::cout << maskBits << std::endl;

  std::cout << config.categoryBits << std::endl;
  std::cout << config.maskBits << std::endl;
  swarm::CollisionConfig treeConfig =
      swarm::CollisionManager::getCollisionConfig(typeid(swarm::Tree));
  std::bitset<16> c(treeConfig.categoryBits);
  std::cout << c << std::endl;
  std::bitset<16> m(treeConfig.maskBits);
  std::cout << m << std::endl;
  auto contactListener = std::make_shared<swarm::BaseContactListener>();
  setupInteractions(*contactListener);
  SwarmTest::SetContactListener(*contactListener);
  SwarmTest::AddBehaviour("Flocking", std::move(flock));
  SwarmTest::AddBehaviour("Pheromone", std::move(pheromone));
  SwarmTest::SetConfiguration(smallDrone);
  SwarmTest::SetNumDrones(DRONE_COUNT);
  SwarmTest::Run();
  return 0;
}