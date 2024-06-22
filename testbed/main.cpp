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

#include "behaviours/registry.h"
#include "core/test_queue.h"
#include "draw.h"
#include "drones/drone.h"
#include "drones/drone_configuration.h"
#include "drones/drone_factory.h"
#include "imgui.h"
#include "nlohmann/json.hpp"
#include "run_sim.h"
#include "settings.h"
#include "test.h"
#include "utils/base_contact_listener.h"
#include "utils/collision_manager.h"
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
#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <limits.h>
#include <unistd.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#endif

std::filesystem::path getExecutablePath() {
#if defined(_WIN32)
  char path[MAX_PATH] = {0};
  HMODULE hModule = GetModuleHandle(nullptr);
  if (GetModuleFileNameA(hModule, path, MAX_PATH) == 0) {
    throw std::runtime_error("Failed to get module filename.");
  }
  return std::filesystem::path(path).parent_path();
#elif defined(__linux__)
  char result[PATH_MAX];
  ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
  if (count == -1) {
    throw std::runtime_error(
        "Failed to read symbolic link for executable path.");
  }
  return std::filesystem::path(std::string(result, (count > 0) ? count : 0))
      .parent_path();
#elif defined(__APPLE__)
  char path[1024];
  uint32_t size = sizeof(path);
  if (_NSGetExecutablePath(path, &size) != 0) {
    throw std::runtime_error("Buffer too small; increase buffer size.");
  }
  return std::filesystem::path(path).parent_path();
#else
  throw std::runtime_error("Unsupported platform.");
#endif
}

b2World *LoadMap(const char *new_map_name) {
  std::filesystem::path exec_path = getExecutablePath();
  std::filesystem::path file_path = exec_path / ".." / ".." / "testbed" /
                                    "maps" /
                                    (std::string(new_map_name) + ".json");

  b2World *world = new b2World(b2Vec2(0.0f, 0.0f));
  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file at: " + file_path.string());
  }

  nlohmann::json map;
  file >> map;

  for (auto &body_json : map["bodies"]) {
    b2BodyDef body_def;
    body_def.type = (b2BodyType)body_json["type"];
    body_def.position.Set(body_json["position"][0], body_json["position"][1]);
    body_def.angle = body_json["angle"];
    body_def.linearDamping = body_json["linear_damping"];
    body_def.angularDamping = body_json["angular_damping"];
    body_def.gravityScale = body_json["gravity_scale"];
    body_def.fixedRotation = body_json["fixed_rotation"];
    body_def.bullet = body_json["bullet"];

    b2Body *body = world->CreateBody(&body_def);
    std::cout << "Created Body: " << body << std::endl;
    std::cout << "Body Type: " << body->GetType() << std::endl;
    std::cout << "Body Position: " << body->GetPosition().x << ", "
              << body->GetPosition().y << std::endl;

    for (auto &fixture_json : body_json["fixtures"]) {
      b2FixtureDef fixture_def;
      fixture_def.density = fixture_json["density"];
      fixture_def.friction = fixture_json["friction"];
      fixture_def.restitution = fixture_json["restitution"];
      fixture_def.isSensor = fixture_json["is_sensor"];
      fixture_def.filter.categoryBits = fixture_json["category_bits"];
      fixture_def.filter.maskBits = fixture_json["mask_bits"];
      fixture_def.filter.groupIndex = fixture_json["group_index"];

      if (fixture_json.find("polygon") != fixture_json.end()) {
        b2PolygonShape shape;
        for (auto &vertex : fixture_json["polygon"]) {
          shape.m_vertices[shape.m_count++] = {vertex[0], vertex[1]};
        }
        fixture_def.shape = &shape;
      } else if (fixture_json.find("circle") != fixture_json.end()) {
        b2CircleShape shape;
        shape.m_p.Set(fixture_json["circle"]["center"][0],
                      fixture_json["circle"]["center"][1]);
        shape.m_radius = fixture_json["circle"]["radius"];
        fixture_def.shape = &shape;
      } else if (fixture_json.find("edge") != fixture_json.end()) {
        b2EdgeShape shape;
        shape.m_vertex1.Set(fixture_json["edge"]["start"][0],
                            fixture_json["edge"]["start"][1]);
        shape.m_vertex2.Set(fixture_json["edge"]["end"][0],
                            fixture_json["edge"]["end"][1]);
        fixture_def.shape = &shape;
      }
      body->CreateFixture(&fixture_def);
    }
  }
  return world;
}
int main() {
  b2World *world = LoadMap("test");
  b2World *world2 = LoadMap("test2");

  auto flock_params = swarm::behaviour::Registry::getInstance()
                          .getBehaviour("Flocking")
                          ->getParameters();

  swarm::DroneConfiguration *smallDrone = new swarm::DroneConfiguration(
      25.0f, 50.0f, 10.0f, 0.3f, 1.0f, 1.5f, 4000.0f);
  swarm::CollisionManager::registerType(typeid(swarm::Drone),
                                        {typeid(swarm::Tree)});
  swarm::CollisionManager::registerType(typeid(swarm::Tree),
                                        {typeid(swarm::Drone)});

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
      "Flocking", pheromone_params, smallDrone, world, 100, 0, 1200.0f,
  };

  swarm::TestConfig config2 = {
      "Pheromone Avoidance",
      pheromone_params,
      smallDrone,
      world,
      100,
      0,
      1200.0f,
  };

  swarm::TestQueue queue;
  config.num_drones = 50;
  queue.push(config);
  queue.push(config2);
  config.num_drones = 50;
  queue.push(config);
  run_sim();
  return 0;
}