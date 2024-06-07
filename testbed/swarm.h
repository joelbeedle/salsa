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

#define TREE_COUNT 50000
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
  static float border_height_;
  static float border_width_;

  swarm::Behaviour *behaviour_;
  std::string current_behaviour_name_;
  swarm::DroneContactListener contact_listener_;

  // Setup drone parameters and configurations
  std::unordered_map<std::string, DroneParameters> all_drone_parameters_;
  std::unordered_map<std::string, swarm::DroneConfiguration>
      all_drone_configurations_;

  static swarm::DroneConfiguration *drone_configuration_;
  DroneParameters *drone_parameters_;

  std::vector<std::unique_ptr<swarm::Drone>> drones_;

  float obstacle_view_range_;
  float camera_view_range_;
  float max_speed_;
  float max_force_;

  static float num_drones_;

  bool draw_visual_range_ = true;

 public:
  SwarmTest() {
    init_world();
    g_camera.m_center.x = border_height_ / 2;
    g_camera.m_center.y = border_width_ / 2;
    g_camera.m_zoom = 10.0f;
    auto &registry = swarm::behaviour::Registry::getInstance();
    auto behaviour_names = registry.getBehaviourNames();
    if (!behaviour_names.empty()) {
      current_behaviour_name_ = behaviour_names[0];
      behaviour_ = registry.getBehaviour(current_behaviour_name_);
    }
    std::cout << num_drones_ << std::endl;
    create_drones(*behaviour_, *drone_configuration_);
  }

  void init_world() {
    create_bounds(m_world);
    g_debugDraw.SetFlags(b2Draw::e_shapeBit | b2Draw::e_jointBit);
    b2Vec2 gravity(0.0f, 0.0f);
    m_world->SetGravity(gravity);
  }

  void create_bounds(b2World *world) {
    // Define the ground body.
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0.0f, 0.0f);

    b2Body *groundBody = world->CreateBody(&groundBodyDef);

    b2EdgeShape groundBox;

    // bottom
    groundBox.SetTwoSided(b2Vec2(0.0f, 0.0f), b2Vec2(border_width_, 0.0f));
    groundBody->CreateFixture(&groundBox, 0.0f);

    // top
    groundBox.SetTwoSided(b2Vec2(0.0f, border_height_),
                          b2Vec2(border_width_, border_height_));
    groundBody->CreateFixture(&groundBox, 0.0f);

    // left
    groundBox.SetTwoSided(b2Vec2(0.0f, 0.0f), b2Vec2(0.0f, border_height_));
    groundBody->CreateFixture(&groundBox, 0.0f);

    // right
    groundBox.SetTwoSided(b2Vec2(border_width_, 0.0f),
                          b2Vec2(border_width_, border_height_));
    groundBody->CreateFixture(&groundBox, 0.0f);
  }

  void create_drones(swarm::Behaviour &b, swarm::DroneConfiguration &config) {
    const float margin = 2.0f;  // Define a margin to prevent spawning exactly
                                // at the border or outside
    for (int i = 0; i < num_drones_; i++) {
      float x =
          (rand() % static_cast<int>(border_width_ - 2 * margin)) + margin;
      float y =
          (rand() % static_cast<int>(border_height_ - 2 * margin)) + margin;
      drones_.push_back(
          swarm::DroneFactory::createDrone(m_world, b2Vec2(x, y), b, config));
    }
  }

  void SetBehaviour() {
    for (auto &drone : drones_) {
      drone->setBehaviour(*behaviour_);
    }
  }

  void UpdateDroneSettings() {
    for (auto &drone : drones_) {
      drone->setMaxForce(drone_configuration_->maxForce);
      drone->setMaxSpeed(drone_configuration_->maxSpeed);
      drone->setViewRange(drone_configuration_->cameraViewRange);
      drone->setObstacleViewRange(drone_configuration_->obstacleViewRange);
      drone->updateSensorRange();
    }
  }

  static void AddBehaviour(const std::string &name,
                           std::unique_ptr<swarm::Behaviour> behaviour) {
    swarm::behaviour::Registry::getInstance().add(name, std::move(behaviour));
  }
  auto border_height() const & -> const float & { return border_height_; }
  auto border_height() & -> float & { return border_height_; }
  static void SetHeight(float height) { border_height_ = height; }
  static void SetWidth(float width) { border_width_ = width; }
  static float GetHeight() { return border_height_; }
  static float GetWidth() { return border_width_; }

  static void SetConfiguration(swarm::DroneConfiguration *configuration) {
    // Set the drone configuration
    drone_configuration_ = configuration;
  }

  static void SetNumDrones(float num_drones) { num_drones_ = num_drones; }
  static Test *Create() {
    // Generate a new version of this test with the user defined settings
    return new SwarmTest();
  }
  void Step(Settings &settings) override {
    // Run simulation steps here
    Test::Step(settings);

    for (auto &drone : drones_) {
      drone->update(drones_);
    }

    Draw(m_world, &g_debugDraw, std::vector<int>());
  }
  void UpdateUI() override {
    ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
    ImGui::SetNextWindowSize(ImVec2(285.0f, 285.0f));
    ImGui::Begin("Swarm Controls", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

    if (ImGui::BeginCombo("Behaviours", current_behaviour_name_.c_str())) {
      auto behaviourNames =
          swarm::behaviour::Registry::getInstance().getBehaviourNames();

      for (auto &name : behaviourNames) {
        bool isSelected = (current_behaviour_name_ == name);
        if (ImGui::Selectable(name.c_str(), isSelected)) {
          current_behaviour_name_ = name;
          behaviour_ =
              swarm::behaviour::Registry::getInstance().getBehaviour(name);
          SetBehaviour();
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
    for (auto [name, parameter] : behaviour_->getParameters()) {
      changed |=
          ImGui::SliderFloat(name.c_str(), &(parameter->value()),
                             parameter->min_value(), parameter->max_value());
    }

    if (changed) {
      SetBehaviour();
    }
    ImGui::Separator();
    ImGui::Text("Visual Settings");
    ImGui::Checkbox("Draw Drone visual range", &draw_visual_range_);

    if (ImGui::Button("Reset Simulation")) {
      drones_.clear();
      create_drones(*behaviour_, *drone_configuration_);
    }

    ImGui::End();

    // Drone settings window
    ImGui::SetNextWindowPos(ImVec2(10.0f, 400.0f));
    ImGui::SetNextWindowSize(ImVec2(285.0f, 285.0f));
    ImGui::Begin("Drone Settings", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
    ImGui::Text("Drone Preset Settings");
    bool droneChanged = false;
    droneChanged |= ImGui::SliderFloat(
        "maxSpeed", &drone_configuration_->maxSpeed, 0.0f, 50.0f);
    droneChanged |= ImGui::SliderFloat(
        "maxForce", &drone_configuration_->maxForce, 0.0f, 10.0f);
    droneChanged |= ImGui::SliderFloat("cameraViewRange",
                                       &drone_configuration_->cameraViewRange,
                                       0.0f, 100.0f);
    droneChanged |= ImGui::SliderFloat("obstacleViewRange",
                                       &drone_configuration_->obstacleViewRange,
                                       0.0f, 100.0f);
    droneChanged |= ImGui::SliderFloat(
        "droneDetectionRange", &drone_configuration_->droneDetectionRange, 0.0f,
        4000.0f);

    if (droneChanged) {
      UpdateDroneSettings();
    }

    ImGui::End();
  }
  void Draw(b2World *world, DebugDraw *debugDraw,
            std::vector<int> foundTreeIDs) {
    for (b2Body *body = world->GetBodyList(); body; body = body->GetNext()) {
      const b2Transform &transform = body->GetTransform();

      for (b2Fixture *fixture = body->GetFixtureList(); fixture;
           fixture = fixture->GetNext()) {
        if (fixture->IsSensor()) {
          uint16 categoryBits = fixture->GetFilterData().categoryBits;
          if (categoryBits == 0x0001) {
            // This is a drone sensor, draw if wanted
            if (draw_visual_range_) {
              const b2CircleShape *circleShape =
                  static_cast<const b2CircleShape *>(fixture->GetShape());
              b2Vec2 position =
                  transform.p + b2Mul(transform.q, circleShape->m_p);
              debugDraw->DrawCircle(position, circleShape->m_radius,
                                    b2Color(0.5f, 0.5f, 0.5f));
            }
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
              swarm::Drone *drone = userData->drone;
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

  static void Run() {
    RegisterTest("SwarmTest", "Swarm_Test", SwarmTest::Create);
    run_sim();
  }
};

#endif  // SWARM_H