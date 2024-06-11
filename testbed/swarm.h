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
#include "core/sim.h"
#include "draw.h"
#include "drones/drone.h"
#include "drones/drone_factory.h"
#include "imgui/imgui.h"
#include "run_sim.h"
#include "settings.h"
#include "test.h"
#include "utils/base_contact_listener.h"
#include "utils/collision_manager.h"
#include "utils/drone_configuration.h"
#include "utils/object_types.h"
#include "utils/tree.h"

#define TREE_COUNT 0
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
  swarm::Sim *sim;
  static swarm::SimBuilder *sim_builder;
  bool draw_visual_range_ = true;
  bool draw_trees_ = false;
  bool first_run_ = true;
  std::vector<swarm::Tree *> trees;
  std::vector<b2Vec2> treePositions;
  std::vector<b2Color> treeColors;

  b2Color falseColour =
      b2Color(0.5f * 0.95294f, 0.5f * 0.50588f, 0.5f * 0.50588f, 0.5f * 0.25f);
  b2Color trueColour =
      b2Color(0.5f * 0.77f, 0.5f * 0.92f, 0.5f * 0.66f, 0.5f * 0.25f);

 public:
  SwarmTest() {
    sim_builder = new swarm::SimBuilder();
    sim_builder->setWorld(m_world);
    g_debugDraw.SetFlags(b2Draw::e_shapeBit | b2Draw::e_jointBit);
    // m_world->SetContactListener(contactListener_);
  }

  void Run() {
    auto names = swarm::behaviour::Registry::getInstance().getBehaviourNames();
    sim = sim_builder->build();
    g_camera.m_center.x = sim->world_height() / 2;
    g_camera.m_center.y = sim->world_width() / 2;
    g_camera.m_zoom = 10.0f;

    RunStatic();
  }

  void SetBuilder(swarm::SimBuilder *builder) {
    sim_builder = builder;
    builder->setWorld(m_world);
  }
  void SetHeight(float height) { sim_builder->setWorldHeight(height); }
  void SetWidth(float width) { sim_builder->setWorldWidth(width); }
  float GetHeight() { return sim->world_height(); }
  float GetWidth() { return sim->world_width(); }
  void SetContactListener(swarm::BaseContactListener &listener) {
    sim_builder->setContactListener(listener);
  }
  void AddBehaviour(const std::string &name,
                    std::unique_ptr<swarm::Behaviour> behaviour) {
    swarm::behaviour::Registry::getInstance().add(name, std::move(behaviour));
  }

  void SetConfiguration(swarm::DroneConfiguration *configuration) {
    sim_builder->setDroneConfiguration(configuration);
  }

  swarm::DroneConfiguration *GetConfiguration() {
    return sim->getDroneConfiguration();
  }

  void SetDroneCount(int count) { sim_builder->setDroneCount(count); }

  void createTrees() {
    // Seed for reproducability
    auto seed =
        std::chrono::high_resolution_clock::now().time_since_epoch().count();

    srand(seed);
    const float margin = 2.0f;
    for (int i = 0; i < TREE_COUNT; i++) {
      float x =
          (rand() % static_cast<int>(sim->world_width() - 2 * margin)) + margin;
      float y = (rand() % static_cast<int>(sim->world_height() - 2 * margin)) +
                margin;
      trees.push_back(
          new swarm::Tree(m_world, i, b2Vec2(x, y), false, false, 2.5f));
      treePositions.push_back(trees[i]->getBody()->GetPosition());
      treeColors.push_back(falseColour);
    }
  }

  static Test *Create() {
    // Generate a new version of this test with the user defined settings
    return new SwarmTest();
  }
  void Step(Settings &settings) override {
    // Run simulation steps here
    Test::Step(settings);
    std::vector<int> foundTreeIDs;

    // sim->update();
    Draw(m_world, &g_debugDraw, foundTreeIDs);
  }
  void UpdateUI() override {
    ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
    ImGui::SetNextWindowSize(ImVec2(285.0f, 285.0f));
    ImGui::Begin("Swarm Controls", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

    std::cout << sim->getBehaviourName() << std::endl;
    if (ImGui::BeginCombo("Behaviours",
                          sim->current_behaviour_name().c_str())) {
      auto behaviourNames =
          swarm::behaviour::Registry::getInstance().getBehaviourNames();

      for (auto &name : behaviourNames) {
        bool isSelected = (sim->current_behaviour_name() == name);
        if (ImGui::Selectable(name.c_str(), isSelected)) {
          sim->current_behaviour_name() = name;
          sim->setCurrentBehaviour(name);
          sim->applyCurrentBehaviour();
        }
        if (isSelected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    ImGui::Separator();

    ImGui::Text("Behaviour Settings");
    bool changed = false;
    for (auto [name, parameter] : sim->behaviour()->getParameters()) {
      changed |=
          ImGui::SliderFloat(name.c_str(), &(parameter->value()),
                             parameter->min_value(), parameter->max_value());
    }

    if (changed) {
      sim->applyCurrentBehaviour();
    }
    ImGui::Separator();
    ImGui::Text("Visual Settings");
    ImGui::Checkbox("Draw Drone visual range", &draw_visual_range_);
    ImGui::Checkbox("Draw Trees", &draw_trees_);

    if (ImGui::Button("Reset Simulation")) {
      sim->reset();
    }

    ImGui::End();
    if (sim->getDroneConfiguration() == nullptr) {
      sim->setDroneConfiguration(new swarm::DroneConfiguration(
          25.0f, 50.0f, 10.0f, 0.3f, 1.0f, 1.5f, 4000.0f));
    }

    // Drone settings window
    ImGui::SetNextWindowPos(ImVec2(10.0f, 400.0f));
    ImGui::SetNextWindowSize(ImVec2(285.0f, 285.0f));
    ImGui::Begin("Drone Settings", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
    ImGui::Text("Drone Preset Settings");
    bool droneChanged = false;
    droneChanged |= ImGui::SliderFloat(
        "maxSpeed", &sim->getDroneConfiguration()->maxSpeed, 0.0f, 50.0f);
    droneChanged |= ImGui::SliderFloat(
        "maxForce", &sim->getDroneConfiguration()->maxForce, 0.0f, 10.0f);
    droneChanged |= ImGui::SliderFloat(
        "cameraViewRange", &sim->getDroneConfiguration()->cameraViewRange, 0.0f,
        100.0f);
    droneChanged |= ImGui::SliderFloat(
        "obstacleViewRange", &sim->getDroneConfiguration()->obstacleViewRange,
        0.0f, 100.0f);
    droneChanged |= ImGui::SliderFloat(
        "droneDetectionRange",
        &sim->getDroneConfiguration()->droneDetectionRange, 0.0f, 4000.0f);

    if (droneChanged) {
      sim->updateDroneSettings();
    }

    ImGui::End();
  }
  void Draw(b2World *world, DebugDraw *debugDraw,
            std::vector<int> foundTreeIDs) {
    if (draw_trees_) {
      if (first_run_) {
        debugDraw->DrawAllTrees(treePositions, treeColors);
        first_run_ = false;
      } else {
        debugDraw->DrawTrees(treePositions, treeColors, foundTreeIDs);
      }
    }
    for (b2Body *body = world->GetBodyList(); body; body = body->GetNext()) {
      const b2Transform &transform = body->GetTransform();

      for (b2Fixture *fixture = body->GetFixtureList(); fixture;
           fixture = fixture->GetNext()) {
        if (fixture->IsSensor()) {
          uint16 categoryBits = fixture->GetFilterData().categoryBits;
          if (categoryBits == swarm::CollisionManager::getCollisionConfig(
                                  typeid(swarm::Drone))
                                  .categoryBits &&
              draw_visual_range_) {
            // This is a drone sensor, draw if wanted
            const b2CircleShape *circleShape =
                static_cast<const b2CircleShape *>(fixture->GetShape());
            b2Vec2 position =
                transform.p + b2Mul(transform.q, circleShape->m_p);
            debugDraw->DrawCircle(position, circleShape->m_radius,
                                  b2Color(0.5f, 0.5f, 0.5f));

            // skip this fixture as it's been dealt with
            continue;
          }
          if (categoryBits == 0x0002) {
            // This is fixture tree sensor, but we deal with trees individually
            // later
          }
        }

        if (fixture->GetUserData().pointer != 0) {
          swarm::UserData *userData = reinterpret_cast<swarm::UserData *>(
              fixture->GetUserData().pointer);
          if (userData == nullptr) {
            std::cout << "User data is null" << std::endl;
            continue;
          }
          // Depending on the type, draw the object
          switch (userData->type) {
            case swarm::ObjectType::Drone: {
              swarm::Drone *drone = userData->as<swarm::Drone>();
              // Draw drone
              b2Vec2 position = body->GetPosition();
              debugDraw->DrawSolidCircle(position, drone->getRadius(),
                                         transform.q.GetXAxis(),
                                         b2Color(0.7f, 0.5f, 0.5f));
              break;
            }
            case swarm::ObjectType::Tree: {
              break;
            }
          }
          continue;
        }

        // Draw everything else that's not anything above with default values
        switch (fixture->GetType()) {
          case b2Shape::e_circle: {
            const b2CircleShape *circleShape =
                static_cast<const b2CircleShape *>(fixture->GetShape());
            b2Vec2 position =
                transform.p + b2Mul(transform.q, circleShape->m_p);
            debugDraw->DrawSolidCircle(position, circleShape->m_radius,
                                       transform.q.GetXAxis(),
                                       b2Color(1.0f, 0.5f, 0.5f));
            break;
          }
          case b2Shape::e_polygon: {
            const b2PolygonShape *polygonShape =
                static_cast<const b2PolygonShape *>(fixture->GetShape());
            b2Vec2 vertices[b2_maxPolygonVertices];
            for (int i = 0; i < polygonShape->m_count; ++i) {
              vertices[i] = b2Mul(transform, polygonShape->m_vertices[i]);
            }
            debugDraw->DrawSolidPolygon(vertices, polygonShape->m_count,
                                        b2Color(0.5f, 0.5f, 0.5f));
            break;
          }
          case b2Shape::e_edge: {
            const b2EdgeShape *edgeShape =
                static_cast<const b2EdgeShape *>(fixture->GetShape());
            b2Vec2 v1 = b2Mul(transform, edgeShape->m_vertex1);
            b2Vec2 v2 = b2Mul(transform, edgeShape->m_vertex2);
            debugDraw->DrawSegment(v1, v2, b2Color(0.5f, 1.0f, 0.5f));
            break;
          }
          case b2Shape::e_chain: {
            const b2ChainShape *chainShape =
                static_cast<const b2ChainShape *>(fixture->GetShape());
            int32 count = chainShape->m_count;
            const b2Vec2 *vertices = chainShape->m_vertices;
            b2Vec2 v1 = b2Mul(transform, vertices[0]);
            for (int32 i = 1; i < count; ++i) {
              b2Vec2 v2 = b2Mul(transform, vertices[i]);
              debugDraw->DrawSegment(v1, v2, b2Color(0.5f, 0.5f, 0.5f));
              debugDraw->DrawCircle(v1, 0.05f, b2Color(0.5f, 0.5f, 0.5f));
              v1 = v2;
            }
            break;
          }
          default:
            // Shouldn't reach here
            break;
        }
      }
    }
  }

  static void RunStatic() {
    RegisterTest("SwarmTest", "Swarm_Test", SwarmTest::Create);
    run_sim();
  }
};

#endif  // SWARM_H