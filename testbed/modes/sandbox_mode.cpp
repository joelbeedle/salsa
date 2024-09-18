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
#include "ui/ui_builder.h"
#include "ui/components/behaviour_settings_component.h"
#include "ui/components/drone_configuration_component.h"
#include "ui/components/sandbox_settings_component.h"
#include "ui/components/visual_settings_component.h"
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
    auto *smallDrone = new salsa::DroneConfiguration(
        "sandbox_default", 25.0f, 50.0f, 10.0f, 0.3f, 1.0f, 1.5f, 4000.0f);

    g_debugDraw.SetFlags(b2Draw::e_shapeBit | b2Draw::e_jointBit);
    m_world = new b2World(b2Vec2(0.0f, 0.0f));
    sim = new salsa::Sim(m_world, 1, 0, smallDrone, 2000, 2000, 1000.0);
    new_count = 1;

    const auto behaviour_names = salsa::behaviour::Registry::get().behaviour_names();
    sim->setCurrentBehaviour(behaviour_names[0]);
    // m_world->SetContactListener(contactListener_);
  }
  static std::unique_ptr<Test> Create() {
    return std::make_unique<SandboxSimulator>();
  }
  static void Build() { sim_builder->build(); }
  void SetBuilder(salsa::SimBuilder *builder)const {
    sim_builder = builder;
    builder->setWorld(m_world);
  }

  void UseQueue(salsa::TestQueue stack) {
    using_queue_ = true;
    queue_ = stack;
  }

  bool SetNextTestFromQueue() {
    auto config = salsa::TestQueue::pop();
    const auto temp_sim = new salsa::Sim(config);
    if (temp_sim == nullptr) {
      return false;
    }
    const auto old_sim = sim;
    sim = temp_sim;
    delete old_sim;
    sim->setCurrentBehaviour(sim->current_behaviour_name());
    m_world = sim->getWorld();
    pause = false;
    return true;
  }
  static void SetWorld(b2World *world) { sim_builder->setWorld(world); }
  static void SetHeight(const float height) { sim_builder->setWorldHeight(height); }
  static void SetWidth(const float width) { sim_builder->setWorldWidth(width); }
  [[nodiscard]] float GetHeight()const { return sim->world_height(); }
  [[nodiscard]] float GetWidth()const { return sim->world_width(); }
  static void SetContactListener(salsa::BaseContactListener &listener) {
    sim_builder->setContactListener(listener);
  }
  static void AddBehaviour(const std::string &name,
                    std::unique_ptr<salsa::Behaviour> behaviour) {
    salsa::behaviour::Registry::get().add(name, std::move(behaviour));
  }

  static void SetConfiguration(salsa::DroneConfiguration *configuration) {
    sim_builder->setDroneConfiguration(configuration);
  }

  [[nodiscard]] salsa::DroneConfiguration *GetConfiguration()const {
    return sim->getDroneConfiguration();
  }

  static void SetDroneCount(const int count) { sim_builder->setDroneCount(count); }

  void Step(Settings &settings) override {
    // Run simulation steps here
    Test::Step(settings);
    const float timeStep =
        settings.m_hertz > 0.0f ? 1.0f / settings.m_hertz : 0.0f;
    pause = settings.m_pause;
     constexpr std::vector<int> foundTreeIDs;
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
    UIBuilder builder;

    builder.setTitle("Swarm Controls")
           .setPosition(ImVec2(10.0f, 50.0f));

    builder.addComponent(std::make_unique<BehaviorSettingsComponent>(sim))
           .addComponent(std::make_unique<VisualSettingsComponent>(draw_visual_range_, draw_targets_, sim))
           .addComponent(std::make_unique<DroneSettingsComponent>(sim))
           .addComponent(std::make_unique<SimulationSettingsComponent>(sim, pause, update_drone_count_, new_count, m_world, g_camera));

    builder.render();
  }

  void Draw(b2World *world, DebugDraw *debugDraw,
            const std::vector<int>& foundTreeIDs) const {
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
             const auto circleShape =
                dynamic_cast<const b2CircleShape *>(fixture->GetShape());
            b2Vec2 position =
                transform.p + b2Mul(transform.q, circleShape->m_p);
            debugDraw->DrawCircle(position, circleShape->m_radius,
                                  b2Color(0.5f, 0.5f, 0.5f));

            // skip this fixture as it's been dealt with
            continue;
          }
        }

        if (fixture->GetUserData().pointer != 0) {
          auto *userData = reinterpret_cast<salsa::UserData *>(
              fixture->GetUserData().pointer);
          if (userData == nullptr) {
            std::cout << "User data is null" << std::endl;
            continue;
          }
          // Draw Drones
          std::string name = salsa::type(*(userData->object));
          if (name == "salsa::Drone") {
            const auto *drone = userData->as<salsa::Drone>();
            // Draw drone
            b2Vec2 position = body->GetPosition();
            debugDraw->DrawSolidCircle(position, drone->radius(),
                                       transform.q.GetXAxis(), drone->color());
          } else if (name =="b2_groundBody" && draw_targets_) {
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
            const auto *circleShape =
                dynamic_cast<const b2CircleShape *>(fixture->GetShape());
            b2Vec2 position =
                transform.p + b2Mul(transform.q, circleShape->m_p);
            debugDraw->DrawSolidCircle(position, circleShape->m_radius,
                                       transform.q.GetXAxis(),
                                       b2Color(0.5f, 0.5f, 0.5f));
            break;
          }
          case b2Shape::e_polygon: {
            const auto *polygonShape =
                dynamic_cast<const b2PolygonShape *>(fixture->GetShape());
            b2Vec2 vertices[b2_maxPolygonVertices];
            for (int i = 0; i < polygonShape->m_count; ++i) {
              vertices[i] = b2Mul(transform, polygonShape->m_vertices[i]);
            }
            debugDraw->DrawSolidPolygon(vertices, polygonShape->m_count,
                                        b2Color(0.5f, 1.0f, 0.5f));
            break;
          }
          case b2Shape::e_edge: {
            const auto *edgeShape =
                dynamic_cast<const b2EdgeShape *>(fixture->GetShape());
            b2Vec2 v1 = b2Mul(transform, edgeShape->m_vertex1);
            b2Vec2 v2 = b2Mul(transform, edgeShape->m_vertex2);
            debugDraw->DrawSegment(v1, v2, b2Color(0.5f, 1.0f, 0.5f));
            break;
          }
          case b2Shape::e_chain: {
            const auto *chainShape =
                dynamic_cast<const b2ChainShape *>(fixture->GetShape());
            const int32 count = chainShape->m_count;
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
