#include <box2d/box2d.h>
#include <plot/plot.h>
#include <salsa/salsa.h>
#include <cstdio>

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
#include "ui/components/add_test_modal.h"
#include "ui/components/add_test_permutation_modal.h"
#include "ui/components/drone_configuration_component.h"
#include "ui/components/graph_plotting_settings_compoment.h"
#include "ui/components/test_queue_component.h"
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
class QueueSimulator final : public Test {
  salsa::Sim *sim;
  static salsa::SimBuilder *sim_builder;
  bool draw_visual_range_ = true;
  bool draw_drone_sensor_range_ = true;
  bool draw_targets_ = true;
  bool first_run_ = true;
  bool using_queue_ = true;
  bool pause = false;
  bool next_frame = false;
  bool queue_empty = true;
  bool skipped_test = false;
  salsa::TestQueue queue_;
  std::vector<std::unique_ptr<salsa::Target>> targets;
  std::vector<b2Vec2> target_positions_;
  std::vector<b2Color> target_colors_;
  float target_radius_ = 10.0f;
  bool add_new_test_ = false;
  bool added_new_test_ = false;
  bool add_test_permutation_ = false;
  bool added_test_permutation_ = false;
  b2Color falseColour =
      b2Color(0.5f * 0.95294f, 0.5f * 0.50588f, 0.5f * 0.50588f, 0.5f * 0.25f);
  b2Color trueColour =
      b2Color(0.5f * 0.77f, 0.5f * 0.92f, 0.5f * 0.66f, 0.5f * 0.25f);

 public:
  QueueSimulator() {
    pause = true;
    auto *smallDrone = new salsa::DroneConfiguration(
        "hidden", 25.0f, 50.0f, 10.0f, 0.3f, 1.0f, 1.5f, 4000.0f);

    g_debugDraw.SetFlags(b2Draw::e_shapeBit | b2Draw::e_jointBit);
    m_world = new b2World(b2Vec2(0.0f, 0.0f));
    sim = new salsa::Sim(m_world, 0, 0, smallDrone, 0, 0, 0);

    auto &registry = salsa::behaviour::Registry::get();
    auto behaviour_names = registry.behaviour_names();
    sim->setCurrentBehaviour(behaviour_names[0]);
  }
  static std::unique_ptr<Test> Create() {
    return std::make_unique<QueueSimulator>();
  }

  static void Build() { sim_builder->build(); }

  void SetBuilder(salsa::SimBuilder *builder)const {
    sim_builder = builder;
    builder->setWorld(m_world);
  }

  void UseQueue(const salsa::TestQueue stack) {
    using_queue_ = true;
    queue_ = stack;
  }

  bool SetNextTestFromQueue() {
    auto config = salsa::TestQueue::pop();
    const auto temp_sim = new salsa::Sim(config);
    const auto old_sim = sim;
    sim = temp_sim;
    if (const std::string current_log_file = old_sim->getCurrentLogFile(); !current_log_file.empty() && !skipped_test)
      testbed::plot(current_log_file);
    delete old_sim;
    sim->setCurrentBehaviour(sim->current_behaviour_name());
    m_world = sim->getWorld();

    g_camera.m_center = sim->getMap().drone_spawn_point;
    g_camera.m_zoom = 10.0f;
    pause = false;
    first_run_ = true;
    skipped_test = false;
    target_positions_.clear();
    target_colors_.clear();
    const auto targets = sim->getTargets();
    const int size = targets.size();
    target_positions_.reserve(size);
    target_colors_.reserve(size);
    bool radius_set = false;
    for (auto &target : targets) {
      if (!radius_set) {
        target_radius_ = target->radius();
        radius_set = true;
      }
      target_positions_.push_back(target->position());
      target_colors_.push_back(falseColour);
    }

    return true;
  }
  static void SetWorld(b2World *world) { sim_builder->setWorld(world); }
  static void SetHeight(float height) { sim_builder->setWorldHeight(height); }
  static void SetWidth(float width) { sim_builder->setWorldWidth(width); }
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

  void generatePermutations(std::vector<std::vector<float>> &results,
                            const std::vector<std::vector<float>> &lists,
                            std::vector<float> current = {}, size_t depth = 0) {
    if (depth == lists.size()) {
      results.push_back(current);
      return;
    }

    for (const auto &item : lists[depth]) {
      current.push_back(item);
      generatePermutations(results, lists, current, depth + 1);
      current.pop_back();
    }
  }

  // Function to parse a list of values from a string
  static std::vector<float> parseList(const std::string &str) {
    std::vector<float> list;
    std::istringstream iss(str);
    std::string value;
    while (std::getline(iss, value, ' ')) {
      try {
        list.push_back(std::stof(value));
      } catch (const std::invalid_argument &e) {
        std::cerr << "Invalid float: " << value << std::endl;
      }
    }
    return list;
  }

  // Function to generate a list of values from a range
  static std::vector<float> generateRange(const float min, const float max, const float step) {
    std::vector<float> range;
    for (float value = min; value <= max + step / 2; value += step) {
      value = std::round(value * 1e6) / 1e6;  // Reduce precision issues
      if (value <= max) {
        range.push_back(value);
      }
    }
    return range;
  }

  void Step(Settings &settings) override {
    // Run simulation steps here
    Test::Step(settings);
    pause = settings.m_pause;
    const std::vector<int> foundIds;
    for (int i = 0; i < settings.m_simulationSpeed; i++) {
      if (!pause) sim->update();
    }

    for (const auto &target : sim->getTargetsFoundThisStep()) {
      target_colors_[target->id()] = trueColour;
    }
    if (queue_empty) {
      pause = true;
      queue_empty = salsa::TestQueue::isEmpty();
    }
    if (!pause)
      sim->current_time() +=
          (1.0f / settings.m_hertz) * settings.m_simulationSpeed;
    Draw(sim->getWorld(), &g_debugDraw, foundIds);
    if (next_frame) {
      pause = true;
      try {
        SetNextTestFromQueue();
        queue_empty = false;
      } catch (const std::exception &e) {
        spdlog::error("Error: {}", e.what());
        std::cerr << e.what() << std::endl;
        pause = true;
        queue_empty = true;
      }
      next_frame = false;
    }
    if (!queue_empty && sim->current_time() >= sim->time_limit()) {
      // This sim is finished, get the next one from the stack pause = true;
      next_frame = true;
    }

    g_debugDraw.DrawString(5, m_textLine, "%fs", sim->current_time());
    m_textLine += m_textIncrement;
  }

  void UpdateUI() override {
    // Create the UIBuilder
    UIBuilder builder;

    // Set window title and position
    builder.setTitle("Swarm Controls")
           .setPosition(ImVec2(10.0f, 50.0f));

    builder.addComponent(std::make_unique<BehaviorSettingsComponent>(sim))   // Behaviour settings
       .addComponent(std::make_unique<VisualSettingsComponent>(draw_visual_range_, draw_targets_, sim))   // Visual settings
       .addComponent(std::make_unique<TestQueueComponent>(sim, pause, next_frame, skipped_test, add_new_test_, add_test_permutation_))   // Test queue
       .addComponent(std::make_unique<GraphPlottingSettingsComponent>())   // Graph plotting settings
       .addComponent(std::make_unique<DroneSettingsComponent>(sim));   // Drone settings

    // Render the UI
    builder.render();

  }

  void Draw(b2World *world, DebugDraw *debugDraw,
            const std::vector<int>& foundTreeIDs) {
    if (draw_targets_) {
      if (first_run_) {
        debugDraw->DrawAllTargets(target_positions_, target_colors_,
                                  target_radius_);
        first_run_ = false;
      } else {
        debugDraw->DrawTargets(target_positions_, target_colors_, foundTreeIDs);
      }
    }
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
            const auto *circleShape =
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
          } else if (name == "b2_groundBody" && draw_targets_) {
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
             const auto  circleShape =
                dynamic_cast<const b2CircleShape *>(fixture->GetShape());
            b2Vec2 position =
                transform.p + b2Mul(transform.q, circleShape->m_p);
            debugDraw->DrawSolidCircle(position, circleShape->m_radius,
                                       transform.q.GetXAxis(),
                                       b2Color(0.5f, 1.0f, 0.5f));
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
static int testIndex =
    RegisterTest("Simulator", "Queue Mode", QueueSimulator::Create);
