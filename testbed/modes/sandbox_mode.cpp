#include <box2d/box2d.h>
#include <salsa/salsa.h>
#include <stdio.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "draw.h"
#include "imgui.h"
#include "settings.h"
#include "test.h"
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
class SandboxSimulator : public Test {
 private:
  salsa::Sim *sim;
  static salsa::SimBuilder *sim_builder;
  bool draw_visual_range_ = true;
  bool draw_drone_sensor_range_ = true;
  bool draw_targets_ = false;
  bool first_run_ = true;
  bool using_queue_ = true;
  bool pause = false;
  bool next_frame = false;
  salsa::TestQueue queue_;
  std::vector<salsa::Target *> targets;
  std::vector<b2Vec2> treePositions;
  std::vector<b2Color> treeColors;

  bool add_new_test_ = false;
  bool added_new_test_ = false;
  bool add_test_permutation_ = false;
  bool added_test_permutation_ = false;
  bool update_drone_count_ = false;
  int new_count;
  b2Color falseColour =
      b2Color(0.5f * 0.95294f, 0.5f * 0.50588f, 0.5f * 0.50588f, 0.5f * 0.25f);
  b2Color trueColour =
      b2Color(0.5f * 0.77f, 0.5f * 0.92f, 0.5f * 0.66f, 0.5f * 0.25f);

 public:
  SandboxSimulator() {
    salsa::DroneConfiguration *smallDrone = new salsa::DroneConfiguration(
        "sandbox_default", 25.0f, 50.0f, 10.0f, 0.3f, 1.0f, 1.5f, 4000.0f);

    g_debugDraw.SetFlags(b2Draw::e_shapeBit | b2Draw::e_jointBit);
    m_world = new b2World(b2Vec2(0.0f, 0.0f));
    sim = new salsa::Sim(m_world, 1, 0, smallDrone, 2000, 2000, 1000.0);
    new_count = 1;

    auto behaviour_names = salsa::behaviour::Registry::get().behaviour_names();
    sim->setCurrentBehaviour(behaviour_names[0]);
    // m_world->SetContactListener(contactListener_);
  }
  static std::unique_ptr<Test> Create() {
    return std::make_unique<SandboxSimulator>();
  }
  void Build() { sim_builder->build(); }
  void SetBuilder(salsa::SimBuilder *builder) {
    sim_builder = builder;
    builder->setWorld(m_world);
  }

  void UseQueue(salsa::TestQueue stack) {
    using_queue_ = true;
    queue_ = stack;
  }

  bool SetNextTestFromQueue() {
    auto config = queue_.pop();
    auto temp_sim = new salsa::Sim(config);
    if (temp_sim == nullptr) {
      return false;
    }
    auto old_sim = sim;
    sim = temp_sim;
    delete old_sim;
    sim->setCurrentBehaviour(sim->current_behaviour_name());
    m_world = sim->getWorld();
    pause = false;
    return true;
  }
  void SetWorld(b2World *world) { sim_builder->setWorld(world); }
  void SetHeight(float height) { sim_builder->setWorldHeight(height); }
  void SetWidth(float width) { sim_builder->setWorldWidth(width); }
  float GetHeight() { return sim->world_height(); }
  float GetWidth() { return sim->world_width(); }
  void SetContactListener(salsa::BaseContactListener &listener) {
    sim_builder->setContactListener(listener);
  }
  void AddBehaviour(const std::string &name,
                    std::unique_ptr<salsa::Behaviour> behaviour) {
    salsa::behaviour::Registry::get().add(name, std::move(behaviour));
  }

  void SetConfiguration(salsa::DroneConfiguration *configuration) {
    sim_builder->setDroneConfiguration(configuration);
  }

  salsa::DroneConfiguration *GetConfiguration() {
    return sim->getDroneConfiguration();
  }

  void SetDroneCount(int count) { sim_builder->setDroneCount(count); }

  void Step(Settings &settings) override {
    // Run simulation steps here
    Test::Step(settings);
    float timeStep =
        settings.m_hertz > 0.0f ? 1.0f / settings.m_hertz : float(0.0f);
    pause = settings.m_pause;
    std::vector<int> foundTreeIDs;
    for (int i = 0; i < settings.m_simulationSpeed; i++) {
      m_world->Step(timeStep, settings.m_velocityIterations,
                    settings.m_positionIterations);
      sim->update();
      if (timeStep > 0.0f) {
        ++m_stepCount;
      }
    }
    if (!settings.m_pause) sim->current_time() += 1.0f / settings.m_hertz;
    Draw(sim->getWorld(), &g_debugDraw, foundTreeIDs);
    if (update_drone_count_) {
      sim->setDroneCount(new_count);
      sim->reset();
      update_drone_count_ = false;
      pause = false;
    }
    // g_debugDraw.DrawString(5, m_textLine, "%fs", sim->current_time());
    // m_textLine += m_textIncrement;
  }

  void UpdateUI() override {
    ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
    ImGui::Begin("Swarm Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    if (ImGui::CollapsingHeader("Behaviour Settings")) {
      ImGui::SeparatorText("Current Behaviour");
      if (ImGui::BeginCombo("Behaviours",
                            sim->current_behaviour_name().c_str())) {
        auto behaviourNames =
            salsa::behaviour::Registry::get().behaviour_names();

        for (auto &name : behaviourNames) {
          bool isSelected = (sim->current_behaviour_name() == name);
          if (ImGui::Selectable(name.c_str(), isSelected)) {
            sim->current_behaviour_name() = name;
            sim->setCurrentBehaviour(name);
          }
          if (isSelected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }

      ImGui::SeparatorText("Behaviour Settings");
      bool changed = false;
      auto behaviour = salsa::behaviour::Registry::get().behaviour(
          sim->current_behaviour_name());
      for (auto [name, parameter] : behaviour->getParameters()) {
        changed |=
            ImGui::SliderFloat(name.c_str(), &(parameter->value()),
                               parameter->min_value(), parameter->max_value());
      }

      if (changed) {
        sim->setCurrentBehaviour(sim->current_behaviour_name());
      }
      ImGui::SeparatorText("Visual Settings");
      ImGui::Checkbox("Draw Drone visual range", &draw_visual_range_);
      ImGui::Checkbox("Draw Targets", &draw_targets_);

      if (ImGui::Button("Reset Simulation")) {
        sim->reset();
      }
    }

    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    if (ImGui::CollapsingHeader("Drone Settings")) {
      // Drone settings window
      ImGui::SeparatorText("Drone Preset Settings");
      bool droneChanged = false;
      droneChanged |= ImGui::SliderFloat(
          "maxSpeed", &sim->getDroneConfiguration()->maxSpeed, 0.0f, 50.0f);
      droneChanged |= ImGui::SliderFloat(
          "maxForce", &sim->getDroneConfiguration()->maxForce, 0.0f, 10.0f);
      droneChanged |= ImGui::SliderFloat(
          "cameraViewRange", &sim->getDroneConfiguration()->cameraViewRange,
          0.0f, 100.0f);
      droneChanged |= ImGui::SliderFloat(
          "obstacleViewRange", &sim->getDroneConfiguration()->obstacleViewRange,
          0.0f, 100.0f);
      droneChanged |= ImGui::SliderFloat(
          "droneDetectionRange",
          &sim->getDroneConfiguration()->droneDetectionRange, 0.0f, 4000.0f);

      if (droneChanged) {
        sim->updateDroneSettings();
      }
    }
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    if (ImGui::CollapsingHeader("Simulation Settings")) {
      ImGui::SeparatorText("Simulation Settings");

      // Change drone count
      static int changed_count = new_count;
      bool changed = ImGui::SliderInt("Drone Count", &new_count, 0, 100);
      if (changed) {
        changed_count = new_count;
        pause = true;
        update_drone_count_ = true;
      }

      auto mapNames = salsa::map::getMapNames();
      static std::string current_map_name = mapNames[0];
      if (ImGui::BeginCombo("Map", current_map_name.c_str())) {
        for (auto &name : mapNames) {
          bool isSelected = (current_map_name == name);
          if (ImGui::Selectable(name.c_str(), isSelected)) {
            // set current map in sim and reset
            current_map_name = name;
            sim->changeMap(name);
            m_world = sim->getWorld();
            g_camera.m_center = sim->getDroneSpawnPosition();
            g_camera.m_zoom = 10.0f;
          }
          if (isSelected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }

      auto new_map = salsa::map::getMap(current_map_name);
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
          if (categoryBits ==
                  salsa::CollisionManager::getCollisionConfig<salsa::Drone>()
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
        }

        if (fixture->GetUserData().pointer != 0) {
          salsa::UserData *userData = reinterpret_cast<salsa::UserData *>(
              fixture->GetUserData().pointer);
          if (userData == nullptr) {
            std::cout << "User data is null" << std::endl;
            continue;
          }
          // Draw Drones
          std::string name = salsa::type(*(userData->object));
          if (name.compare("salsa::Drone") == 0) {
            salsa::Drone *drone = userData->as<salsa::Drone>();
            // Draw drone
            b2Vec2 position = body->GetPosition();
            debugDraw->DrawSolidCircle(position, drone->radius(),
                                       transform.q.GetXAxis(), drone->color());
          } else if (name.compare("b2_groundBody") && draw_targets_) {
            // salsa::Target *target = userData->as<salsa::Target>();
            // b2Vec2 position = body->GetPosition();
            // debugDraw->DrawSolidCircle(position, target->getRadius(),
            //                           transform.q.GetXAxis(),
            //                           target->getColor());
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
                                       b2Color(0.5f, 0.5f, 0.5f));
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
};
static int testIndex2 =
    RegisterTest("Simulator", "Sandbox Mode", SandboxSimulator::Create);
