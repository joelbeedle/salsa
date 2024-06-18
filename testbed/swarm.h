#ifndef SWARM_H
#define SWARM_H
#include <box2d/box2d.h>
#include <stdio.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "behaviours/registry.h"
#include "core/sim.h"
#include "core/test_queue.h"
#include "draw.h"
#include "drones/drone.h"
#include "imgui/imgui.h"
#include "map.h"
#include "run_sim.h"
#include "settings.h"
#include "test.h"
#include "utils/base_contact_listener.h"
#include "utils/collision_manager.h"
#include "utils/drone_configuration.h"
#include "utils/object_types.h"
#include "utils/tree.h"

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
  bool draw_drone_sensor_range_ = true;
  bool draw_trees_ = false;
  bool first_run_ = true;
  bool using_queue_ = true;
  bool pause = false;
  bool next_frame = false;
  swarm::TestQueue queue_;
  std::vector<swarm::Tree *> trees;
  std::vector<b2Vec2> treePositions;
  std::vector<b2Color> treeColors;

  bool add_new_test_ = false;
  bool added_new_test_ = false;
  bool add_test_permutation_ = false;
  bool added_test_permutation_ = false;
  b2Color falseColour =
      b2Color(0.5f * 0.95294f, 0.5f * 0.50588f, 0.5f * 0.50588f, 0.5f * 0.25f);
  b2Color trueColour =
      b2Color(0.5f * 0.77f, 0.5f * 0.92f, 0.5f * 0.66f, 0.5f * 0.25f);

 public:
  SwarmTest() {
    g_debugDraw.SetFlags(b2Draw::e_shapeBit | b2Draw::e_jointBit);
    sim_builder->setWorld(new b2World(b2Vec2(0.0f, 0.0f)));
    sim = sim_builder->build();

    auto &registry = swarm::behaviour::Registry::getInstance();
    auto behaviour_names = registry.getBehaviourNames();
    if (!behaviour_names.empty()) {
      sim->current_behaviour_name() = behaviour_names[0];
      sim->setBehaviour(registry.getBehaviour(sim->current_behaviour_name()));
    }
    g_camera.m_center.x = sim->world_height() / 2;
    g_camera.m_center.y = sim->world_width() / 2;
    g_camera.m_zoom = 10.0f;

    // m_world->SetContactListener(contactListener_);
  }
  void Build() { sim_builder->build(); }
  void SetBuilder(swarm::SimBuilder *builder) {
    sim_builder = builder;
    builder->setWorld(m_world);
  }

  void UseQueue(swarm::TestQueue stack) {
    using_queue_ = true;
    queue_ = stack;
  }

  bool SetNextTestFromQueue() {
    auto config = queue_.pop();
    auto temp_sim = new swarm::Sim(config);
    if (temp_sim == nullptr) {
      return false;
    }
    auto old_sim = sim;
    sim = temp_sim;
    delete old_sim;
    sim->setCurrentBehaviour(sim->current_behaviour_name());
    sim->applyCurrentBehaviour();
    m_world = sim->getWorld();
    pause = false;
    return true;
  }
  void SetWorld(b2World *world) { sim_builder->setWorld(world); }
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

  static Test *Create() {
    // Generate a new version of this test with the user defined settings
    return new SwarmTest();
  }
  void Step(Settings &settings) override {
    // Run simulation steps here
    Test::Step(settings);
    settings.m_pause = pause;
    std::vector<int> foundTreeIDs;
    sim->update();
    if (!settings.m_pause) sim->current_time() += 1.0f / settings.m_hertz;
    m_world->DebugDraw();
    Draw(sim->getWorld(), &g_debugDraw, foundTreeIDs);
    if (next_frame) {
      pause = true;
      try {
        SetNextTestFromQueue();
      } catch (const std::exception &e) {
        std::cerr << "No more tests in queue" << std::endl;
        pause = true;
      }
      next_frame = false;
    }
    if (using_queue_) {
      if (sim->current_time() >= sim->time_limit()) {
        // This sim is finished, get the next one from the stack
        pause = true;
        next_frame = true;
      }
    }

    g_debugDraw.DrawString(5, m_textLine, "%fs", sim->current_time());
    m_textLine += m_textIncrement;
  }

  b2World *LoadMap(const char *new_map_name) {
    b2World *world = new b2World(b2Vec2(0.0f, 0.0f));
    std::ifstream file("../../testbed/maps/" + std::string(new_map_name) +
                       ".json");
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
  void UpdateUI() override {
    ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
    ImGui::Begin("Swarm Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    // Modal windows
    if (add_new_test_) {
      pause = true;
      ImGui::OpenPopup("Add Test");
      add_new_test_ = false;
    }

    if (added_new_test_) {
      pause = false;
      added_new_test_ = false;
    }

    if (add_test_permutation_) {
      pause = true;
      ImGui::OpenPopup("Add Test Permutation");
      add_test_permutation_ = false;
    }

    if (added_test_permutation_) {
      pause = false;
      added_test_permutation_ = false;
    }

    ImGuiIO &io = ImGui::GetIO();
    ImGui::SetNextWindowPos(
        ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f),
        ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Add Test", NULL,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
      auto behaviourNames =
          swarm::behaviour::Registry::getInstance().getBehaviourNames();
      static std::string current_name = behaviourNames[0];
      static std::string new_map_name = "";
      static int new_drone_count = 0;
      static int new_target_count = 0;
      static float new_time_limit = 0.0f;
      static bool to_change = false;
      static b2World *new_world = nullptr;
      // Get behaviour name for test
      if (ImGui::BeginCombo("Behaviour", current_name.c_str())) {
        auto behaviourNames =
            swarm::behaviour::Registry::getInstance().getBehaviourNames();

        for (auto &name : behaviourNames) {
          bool isSelected = (current_name == name);
          if (ImGui::Selectable(name.c_str(), isSelected)) {
            current_name = name;
            to_change = true;
          }
          if (isSelected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }

      // Get parameters for test
      auto chosen_behaviour =
          swarm::behaviour::Registry::getInstance().getBehaviour(current_name);
      auto chosen_params = chosen_behaviour->getParameters();
      static std::unordered_map<std::string, swarm::behaviour::Parameter *>
          new_params;
      if (new_params.empty() || to_change) {
        new_params.clear();
        for (const auto &pair : chosen_params) {
          new_params[pair.first] =
              pair.second->clone();  // Clone each Parameter and insert into the
          // new map
          to_change = false;
        }
      }
      for (auto [name, parameter] : new_params) {
        ImGui::SliderFloat(name.c_str(), &(parameter->value()),
                           parameter->min_value(), parameter->max_value());
      }

      // Get world map for test
      static char str1[128] = "";
      ImGui::InputText("Map Name", str1, IM_ARRAYSIZE(str1));
      if (ImGui::Button("Open", ImVec2(120, 0))) {
        new_world = LoadMap(str1);
      }

      if (new_world != nullptr) {
        ImGui::SameLine();
        ImGui::Text("Map Loaded");
      }

      // Get drone configuration for test
      ImGui::Text("Drone Configuration");
      swarm::DroneConfiguration *smallDrone = new swarm::DroneConfiguration(
          25.0f, 50.0f, 10.0f, 0.3f, 1.0f, 1.5f, 4000.0f);

      // Set number of drones
      ImGui::InputInt("Drone Count", &new_drone_count);
      // Set number of targets
      ImGui::InputInt("Target Count", &new_target_count);
      // Set time limit
      ImGui::InputFloat("Time Limit", &new_time_limit);

      // create new_config and add it to the queue
      if (ImGui::Button("Add Test", ImVec2(120, 0))) {
        swarm::TestConfig new_config = {
            current_name,    new_params,       smallDrone,     new_world,
            new_drone_count, new_target_count, new_time_limit,
        };
        queue_.push(new_config);
        added_new_test_ = true;
        ImGui::CloseCurrentPopup();
      }
      ImGui::SameLine();
      if (ImGui::Button("Cancel", ImVec2(120, 0))) {
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }
    ImGui::SetNextWindowPos(
        ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f),
        ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Add Test Permutation", NULL,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
      auto behaviourNames =
          swarm::behaviour::Registry::getInstance().getBehaviourNames();
      static std::string current_name = behaviourNames[0];
      static std::string new_map_name = "";
      static int new_drone_count = 0;
      static int new_target_count = 0;
      static float new_time_limit = 0.0f;
      static bool to_change = false;
      static b2World *new_world = nullptr;
      // Get behaviour name for test
      if (ImGui::BeginCombo("Behaviour", current_name.c_str())) {
        auto behaviourNames =
            swarm::behaviour::Registry::getInstance().getBehaviourNames();

        for (auto &name : behaviourNames) {
          bool isSelected = (current_name == name);
          if (ImGui::Selectable(name.c_str(), isSelected)) {
            current_name = name;
            to_change = true;
          }
          if (isSelected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      // Get parameters for test
      auto chosen_behaviour =
          swarm::behaviour::Registry::getInstance().getBehaviour(current_name);
      auto chosen_params = chosen_behaviour->getParameters();
      static std::unordered_map<std::string, swarm::behaviour::Parameter *>
          new_params;
      if (new_params.empty() || to_change) {
        new_params.clear();
        for (const auto &pair : chosen_params) {
          new_params[pair.first] = pair.second->clone();
          to_change = false;
        }
      }
      ImGui::Separator();
      ImGui::Text("Select Range or List for each parameter");
      ImGui::Text("For Range: Input min, max, and step values");
      ImGui::Text("For List: Input a list of values separated by spaces");
      int index = 0;
      static char buf[16][128];
      const char *combo_items[] = {"Range", "List"};
      static std::vector<int> selections;

      if (selections.size() != new_params.size()) {
        selections.resize(new_params.size(), 0);
      }

      for (auto [name, parameter] : new_params) {
        ImGui::PushItemWidth(80.0f);
        std::string comboLabel = "##combo" + std::to_string(index);
        if (ImGui::BeginCombo(comboLabel.c_str(),
                              combo_items[selections[index]])) {
          for (int n = 0; n < IM_ARRAYSIZE(combo_items); n++) {
            bool is_selected = (selections[index] == n);
            if (ImGui::Selectable(combo_items[n], is_selected)) {
              selections[index] = n;
            }
            if (is_selected) {
              ImGui::SetItemDefaultFocus();
            }
          }
          ImGui::EndCombo();
        }
        ImGui::PopItemWidth();
        ImGui::SetItemAllowOverlap();
        ImGui::SameLine();
        static float vec4f[4] = {0.0, 0.0, 0.0, 0.0};
        if (selections[index] == 0) {
          ImGui::InputFloat3(name.c_str(), vec4f);
        } else {
          ImGui::InputText(name.c_str(), buf[index], IM_ARRAYSIZE(buf[index]));
        }
        index++;
      }

      // Get world map for test
      static char str1[128] = "";
      ImGui::InputText("Map Name", str1, IM_ARRAYSIZE(str1));
      if (ImGui::Button("Open", ImVec2(120, 0))) {
        new_world = LoadMap(str1);
      }

      if (new_world != nullptr) {
        ImGui::SameLine();
        ImGui::Text("Map Loaded");
      }

      // Get drone configuration for test
      ImGui::Text("Drone Configuration");
      swarm::DroneConfiguration *smallDrone = new swarm::DroneConfiguration(
          25.0f, 50.0f, 10.0f, 0.3f, 1.0f, 1.5f, 4000.0f);

      // Set number of drones
      ImGui::InputInt("Drone Count", &new_drone_count);
      // Set number of targets
      ImGui::InputInt("Target Count", &new_target_count);
      // Set time limit
      ImGui::InputFloat("Time Limit", &new_time_limit);

      // create new_config and add it to the queue
      if (ImGui::Button("Add Test", ImVec2(120, 0))) {
        swarm::TestConfig new_config = {
            current_name,    new_params,       smallDrone,     new_world,
            new_drone_count, new_target_count, new_time_limit,
        };
        queue_.push(new_config);
        added_test_permutation_ = true;
        ImGui::CloseCurrentPopup();
      }
      ImGui::SameLine();
      if (ImGui::Button("Cancel", ImVec2(120, 0))) {
        ImGui::CloseCurrentPopup();
      }

      ImGui::EndPopup();
    }

    ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
    if (ImGui::CollapsingHeader("Behaviour Settings")) {
      ImGui::Text("Current Behaviour");
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
      auto behaviour = swarm::behaviour::Registry::getInstance().getBehaviour(
          sim->getBehaviourName());
      for (auto [name, parameter] : behaviour->getParameters()) {
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
      ImGui::Separator();
      ImGui::Text("Simulation Queue");
      ImGui::BeginChild("Current Test", ImVec2(200, 100), true,
                        ImGuiWindowFlags_None | ImGuiWindowFlags_MenuBar);
      if (ImGui::BeginMenuBar()) {
        ImGui::MenuItem("Current Test", NULL, false, false);
      }
      ImGui::EndMenuBar();
      swarm::TestConfig current_config = sim->test_config();
      ImGui::Text("Behaviour: %s", current_config.behaviour_name.c_str());
      ImGui::Text("Drone Count: %d", current_config.num_drones);
      ImGui::Text("Target Count: %d", current_config.num_targets);
      ImGui::Text("Time Limit: %f", current_config.time_limit);
      ImGui::EndChild();
      static int selectedTestIndex = -1;
      static swarm::TestConfig *selectedTest = nullptr;

      ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 5.0f);
      ImGui::BeginChild("Test Queue", ImVec2(0, 100), true,
                        ImGuiWindowFlags_None | ImGuiWindowFlags_MenuBar);
      if (ImGui::BeginMenuBar()) {
        ImGui::MenuItem("Test Queue", NULL, false, false);
        if (ImGui::BeginMenu("Options")) {
          if (ImGui::MenuItem("Clear Queue")) {
            queue_.getTests().clear();
          }
          ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("+")) {
          if (ImGui::MenuItem("New Test")) {
            add_new_test_ = true;
            pause = true;
          }
          if (ImGui::MenuItem("New Test Permutation")) {
            add_test_permutation_ = true;
            pause = true;
          }
          ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
      }
      int testIndex = 0;
      auto &tests = queue_.getTests();
      for (int i = 0; i < tests.size(); ++i) {
        auto &test = tests[i];

        if (ImGui::CollapsingHeader(test.behaviour_name.c_str(), &test.keep)) {
          ImGui::Text("Behaviour: %s", test.behaviour_name.c_str());
          ImGui::Text("Drone Count: %d", test.num_drones);
          ImGui::Text("Target Count: %d", test.num_targets);
          ImGui::Text("Time Limit: %f", test.time_limit);
        }
        if (ImGui::IsItemActive() && !ImGui::IsItemHovered()) {
          int n_next = i + (ImGui::GetMouseDragDelta(0).y < 0.f ? -1 : 1);
          if (n_next >= 0 && n_next < tests.size()) {
            std::swap(tests[i], tests[n_next]);
            ImGui::ResetMouseDragDelta();
          }
        }
        // Remove tests from queue if requested
        if (!test.keep) {
          tests.erase(tests.begin() + i);
          i--;  // Adjust loop index to account for the removed element
        }
      }

      ImGui::EndChild();
      ImGui::PopStyleVar();

      if (ImGui::Button("Next Test in Queue")) {
        pause = true;
        next_frame = true;
      }
      if (sim->getDroneConfiguration() == nullptr) {
        sim->setDroneConfiguration(new swarm::DroneConfiguration(
            25.0f, 50.0f, 10.0f, 0.3f, 1.0f, 1.5f, 4000.0f));
      }
    }
    ImGui::SetNextTreeNodeOpen(true, ImGuiCond_Once);
    if (ImGui::CollapsingHeader("Drone Settings")) {
      // Drone settings window
      ImGui::Text("Drone Preset Settings");
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
            // This is fixture tree sensor, but we deal with trees
            // individually later
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
};

#endif  // SWARM_H
