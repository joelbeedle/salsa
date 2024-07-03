#include <box2d/box2d.h>
#include <plot/plot.h>
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
class QueueSimulator : public Test {
 private:
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
    salsa::DroneConfiguration *smallDrone = new salsa::DroneConfiguration(
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
    std::string current_log_file = old_sim->getCurrentLogFile();
    if (current_log_file != "" && !skipped_test)
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
    auto targets = sim->getTargets();
    int size = targets.size();
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
  std::vector<float> parseList(const std::string &str) {
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
  std::vector<float> generateRange(float min, float max, float step) {
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
    float timeStep =
        settings.m_hertz > 0.0f ? 1.0f / settings.m_hertz : float(0.0f);
    pause = settings.m_pause;
    std::vector<int> foundIds;
    for (int i = 0; i < settings.m_simulationSpeed; i++) {
      if (!pause) sim->update();
    }

    for (auto &target : sim->getTargetsFoundThisStep()) {
      target_colors_[target->id()] = trueColour;
    }
    if (queue_empty) {
      pause = true;
      queue_empty = queue_.isEmpty();
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
      auto behaviourNames = salsa::behaviour::Registry::get().behaviour_names();
      static std::string current_name = behaviourNames[0];
      static std::string new_map_name = "";
      static int new_drone_count = 0;
      static int new_target_count = 0;
      static float new_time_limit = 0.0f;
      static bool to_change = false;
      static b2World *new_world = nullptr;
      static std::string new_target_type = "";
      // Get behaviour name for test
      if (ImGui::BeginCombo("Behaviour", current_name.c_str())) {
        auto behaviourNames =
            salsa::behaviour::Registry::get().behaviour_names();

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
          salsa::behaviour::Registry::get().behaviour(current_name);
      auto chosen_params = chosen_behaviour->getParameters();
      static std::unordered_map<std::string, salsa::behaviour::Parameter *>
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

      auto mapNames = salsa::map::getMapNames();
      static std::string current_map_name = mapNames[0];
      if (ImGui::BeginCombo("Map", current_map_name.c_str())) {
        for (auto &name : mapNames) {
          bool isSelected = (current_map_name == name);
          if (ImGui::Selectable(name.c_str(), isSelected)) {
            current_map_name = name;
          }
          if (isSelected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }

      auto listenerNames = salsa::BaseContactListener::getListenerNames();
      static std::string current_listener_name =
          listenerNames.empty() ? "" : listenerNames[0];

      if (ImGui::BeginCombo("Listener Type", current_listener_name.c_str())) {
        for (auto &name : listenerNames) {
          bool isSelected = (current_listener_name == name);
          if (ImGui::Selectable(name.c_str(), isSelected)) {
            current_listener_name = name;
          }
          if (isSelected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }

      // Get drone configuration for test
      auto droneConfigNames =
          salsa::DroneConfiguration::getDroneConfigurationNames();
      static std::string current_drone_config_name = droneConfigNames[0];
      if (ImGui::BeginCombo("Drone Configuration",
                            current_drone_config_name.c_str())) {
        for (auto &name : droneConfigNames) {
          bool isSelected = (current_drone_config_name == name);
          if (ImGui::Selectable(name.c_str(), isSelected)) {
            current_drone_config_name = name;
          }
          if (isSelected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      auto targetNames = salsa::TargetFactory::getTargetNames();
      std::string current_target_name = targetNames[0];
      // Get target type from TargetFactory Registry
      if (ImGui::BeginCombo("Target Type", current_target_name.c_str())) {
        for (auto &name : targetNames) {
          bool isSelected = (current_target_name == name);
          if (ImGui::Selectable(name.c_str(), isSelected)) {
            current_target_name = name;
          }
          if (isSelected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }

      // Set number of drones
      ImGui::InputInt("Drone Count", &new_drone_count);
      // Set number of targets
      ImGui::InputInt("Target Count", &new_target_count);
      // Set time limit
      ImGui::InputFloat("Time Limit", &new_time_limit);

      // create new_config and add it to the queue
      if (ImGui::Button("Add Test", ImVec2(120, 0))) {
        salsa::TestConfig new_config = {
            current_name,
            salsa::Behaviour::convertParametersToFloat(new_params),
            current_drone_config_name,
            current_map_name,
            new_drone_count,
            new_target_count,
            new_time_limit,
            current_target_name,
            current_listener_name};
        queue_.push(new_config);
        added_new_test_ = true;
        ImGui::CloseCurrentPopup();
      }

      ImGui::SameLine();
      if (ImGui::Button("Cancel", ImVec2(120, 0))) {
        added_new_test_ = true;
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }
    ImGui::SetNextWindowPos(
        ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f),
        ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    if (ImGui::BeginPopupModal("Add Test Permutation", NULL,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
      auto behaviourNames = salsa::behaviour::Registry::get().behaviour_names();
      static std::string current_name = behaviourNames[0];
      static std::string new_map_name = "";
      static int new_drone_count = 0;
      static int new_target_count = 0;
      static float new_time_limit = 0.0f;
      static bool to_change = false;
      static b2World *new_world = nullptr;

      // Get behaviour name for test
      if (ImGui::BeginCombo("Behaviour", current_name.c_str())) {
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
          salsa::behaviour::Registry::get().behaviour(current_name);
      auto chosen_params = chosen_behaviour->getParameters();
      static std::unordered_map<std::string, std::string> input_storage;
      static std::vector<std::string> parameter_names;
      static std::vector<int> selections;
      static std::unordered_map<std::string, std::vector<float>> range_storage;

      if (to_change) {
        input_storage.clear();
        parameter_names.clear();
        selections.clear();
        range_storage.clear();
        for (const auto &pair : chosen_params) {
          parameter_names.push_back(pair.first);
          input_storage[pair.first] =
              std::string(128, '\0');  // Initialize with null characters
          selections.push_back(0);
          range_storage[pair.first] = {0.0f, 0.0f, 0.0f};
        }
        to_change = false;
      }

      ImGui::Separator();
      ImGui::Text("Select Range or List for each parameter");
      ImGui::Text("For Range: Input min, max, and step values");
      ImGui::Text("For List: Input a list of values separated by spaces");

      for (size_t i = 0; i < parameter_names.size(); ++i) {
        const auto &name = parameter_names[i];
        ImGui::PushItemWidth(80.0f);
        std::string comboLabel = "##combo" + std::to_string(i);
        if (ImGui::BeginCombo(comboLabel.c_str(),
                              (selections[i] == 0) ? "Range" : "List")) {
          for (int n = 0; n < 2; n++) {
            bool is_selected = (selections[i] == n);
            if (ImGui::Selectable((n == 0) ? "Range" : "List", is_selected)) {
              selections[i] = n;
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

        if (selections[i] == 0) {  // Range
          ImGui::InputFloat3(name.c_str(), range_storage[name].data());
          // Store the range values as a string for later parsing
          std::ostringstream oss;
          oss << range_storage[name][0] << " " << range_storage[name][1] << " "
              << range_storage[name][2];
          input_storage[name] = oss.str();
        } else {  // List
          char buffer[1024];
          std::strncpy(buffer, input_storage[name].c_str(), sizeof(buffer));
          buffer[sizeof(buffer) - 1] = '\0';
          if (ImGui::InputTextWithHint(name.c_str(),
                                       "Enter values separated by spaces",
                                       buffer, sizeof(buffer))) {
            input_storage[name] = buffer;
          }
        }
      }

      // Get world map for test
      auto mapNames = salsa::map::getMapNames();
      static std::string current_map_name = mapNames[0];
      if (ImGui::BeginCombo("Map", current_map_name.c_str())) {
        for (auto &name : mapNames) {
          bool isSelected = (current_map_name == name);
          if (ImGui::Selectable(name.c_str(), isSelected)) {
            current_map_name = name;
          }
          if (isSelected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }

      auto new_map = salsa::map::getMap(current_map_name);

      auto listenerNames = salsa::BaseContactListener::getListenerNames();
      static std::string current_listener_name =
          listenerNames.empty() ? "" : listenerNames[0];

      if (ImGui::BeginCombo("Listener Type", current_listener_name.c_str())) {
        for (auto &name : listenerNames) {
          bool isSelected = (current_listener_name == name);
          if (ImGui::Selectable(name.c_str(), isSelected)) {
            current_listener_name = name;
          }
          if (isSelected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }

      // Get drone configuration for test
      auto droneConfigNames =
          salsa::DroneConfiguration::getDroneConfigurationNames();
      static std::string current_drone_config_name = droneConfigNames[0];
      if (ImGui::BeginCombo("Drone Configuration",
                            current_drone_config_name.c_str())) {
        for (auto &name : droneConfigNames) {
          bool isSelected = (current_drone_config_name == name);
          if (ImGui::Selectable(name.c_str(), isSelected)) {
            current_drone_config_name = name;
          }
          if (isSelected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }

      auto targetNames = salsa::TargetFactory::getTargetNames();
      std::string current_target_name = targetNames[0];
      // Get target type from TargetFactory Registry
      if (ImGui::BeginCombo("Target Type", current_target_name.c_str())) {
        for (auto &name : targetNames) {
          bool isSelected = (current_target_name == name);
          if (ImGui::Selectable(name.c_str(), isSelected)) {
            current_target_name = name;
          }
          if (isSelected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }
      ImGui::InputInt("Drone Count", &new_drone_count);
      ImGui::InputInt("Target Count", &new_target_count);
      ImGui::InputFloat("Time Limit", &new_time_limit);

      static std::vector<salsa::TestConfig> base;
      static bool generated = false;

      if (ImGui::Button("Generate")) {
        base.clear();
        std::vector<std::vector<float>> lists;

        // Parse parameters
        for (size_t i = 0; i < parameter_names.size(); ++i) {
          const auto &name = parameter_names[i];
          if (selections[i] == 0) {  // Range
            std::istringstream iss(input_storage[name]);
            float min, max, step;
            iss >> min >> max >> step;
            lists.push_back(generateRange(min, max, step));
          } else {  // List
            lists.push_back(parseList(input_storage[name]));
          }
        }

        // Generate permutations
        std::vector<std::vector<float>> permutations;
        generatePermutations(permutations, lists);

        for (const auto &combination : permutations) {
          std::ostringstream oss;
          for (float value : combination) {
            oss << value << " ";
          }
          std::cout << oss.str() << std::endl;
        }
        for (const auto &combination : permutations) {
          std::unordered_map<std::string, salsa::behaviour::Parameter *>
              new_params;
          for (size_t j = 0; j < parameter_names.size(); ++j) {
            const auto &name = parameter_names[j];
            new_params[name] = chosen_params[name]->clone();
            *(new_params[name]) = static_cast<float>(combination[j]);
          }
          salsa::TestConfig new_config = {
              current_name,
              salsa::Behaviour::convertParametersToFloat(new_params),
              current_drone_config_name,
              current_map_name,
              new_drone_count,
              new_target_count,
              new_time_limit,
              current_target_name,
              current_listener_name};

          base.push_back(new_config);
          generated = true;
        }
      }

      if (!generated) {
        ImGui::BeginDisabled();
      }

      if (ImGui::Button("Save Permutations", ImVec2(200, 0))) {
        ImGui::OpenPopup("Save Permutations");
      }

      if (!generated) {
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
          ImGui::SetTooltip("Generate Permutations First");
      }

      if (ImGui::Button("Add Permutations to Queue", ImVec2(200, 0))) {
        for (auto config : base) {
          queue_.push(config);
        }
        added_test_permutation_ = true;
        ImGui::CloseCurrentPopup();
      }

      if (!generated) {
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_ForTooltip))
          ImGui::SetTooltip("Generate Permutations First");
      }

      if (ImGui::BeginPopup("Save Permutations")) {
        static char filename[128] = "";
        ImGui::InputText("Filename", filename, 128);
        if (ImGui::Button("Save")) {
          std::vector<salsa::TestConfig> old_tests;
          for (const auto &config : queue_.getTests()) {
            old_tests.push_back(config);
          }
          queue_.getTests().clear();
          for (const auto &config : base) {
            queue_.push(config);
          }
          queue_.save(filename);
          queue_.getTests().clear();
          for (const auto &config : old_tests) {
            queue_.push(config);
          }
          ImGui::CloseCurrentPopup();
        }

        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
          added_test_permutation_ = true;
          ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
      }

      ImGui::SameLine();
      if (ImGui::Button("Cancel", ImVec2(120, 0))) {
        added_test_permutation_ = true;
        ImGui::CloseCurrentPopup();
      }

      ImGui::EndPopup();
    }

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
      ImGui::SeparatorText("Simulation Queue");
      ImGui::BeginChild("Current Test", ImVec2(200, 100), true,
                        ImGuiWindowFlags_None | ImGuiWindowFlags_MenuBar);
      if (ImGui::BeginMenuBar()) {
        ImGui::MenuItem("Current Test", NULL, false, false);
      }
      ImGui::EndMenuBar();

      salsa::TestConfig current_config = sim->test_config();
      ImGui::Text("Behaviour: %s", current_config.behaviour_name.c_str());
      ImGui::Text("Drone Count: %d", current_config.num_drones);
      ImGui::Text("Target Count: %d", current_config.num_targets);
      ImGui::Text("Time Limit: %f", current_config.time_limit);
      ImGui::EndChild();

      ImGui::SameLine();

      ImGui::BeginChild("Next Test", ImVec2(200, 100), true,
                        ImGuiWindowFlags_None | ImGuiWindowFlags_MenuBar);
      if (ImGui::BeginMenuBar()) {
        ImGui::MenuItem("Next Test", NULL, false, false);
      }
      salsa::TestConfig next_config;
      ImGui::EndMenuBar();
      try {
        next_config = queue_.peek();
        ImGui::Text("Behaviour: %s", next_config.behaviour_name.c_str());
        ImGui::Text("Drone Count: %d", next_config.num_drones);
        ImGui::Text("Target Count: %d", next_config.num_targets);
        ImGui::Text("Time Limit: %f", next_config.time_limit);

      } catch (const std::exception &e) {
        ImGui::Text("No more tests in queue");
      }

      ImGui::EndChild();
      static int selectedTestIndex = -1;
      static salsa::TestConfig *selectedTest = nullptr;

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
        skipped_test = true;
      }

      ImGui::SameLine();

      if (ImGui::Button("Save Queue")) {
        ImGui::OpenPopup("Save Queue");
      }

      if (ImGui::BeginPopup("Save Queue")) {
        static char filename[128] = "";
        ImGui::InputText("Filename", filename, 128);
        if (ImGui::Button("Save")) {
          queue_.save(filename);
          ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
      }

      ImGui::SeparatorText("Graph Plotting Settings");

      static bool plot_graphs = true;
      static bool plot_graph1 = false;
      static bool plot_graph2 = false;
      static bool plot_graph3 = false;
      static bool plot_graph4 = false;
      static bool plot_graph5 = false;

      ImGui::Checkbox("Plot Graphs", &plot_graphs);
      if (plot_graphs) {
        ImGui::Checkbox("Plot Targets Found", &testbed::plot_targets_found);
        ImGui::Checkbox("Plot Drone Distances", &testbed::plot_drone_distances);
        ImGui::Checkbox("Plot Drone Heatmap", &testbed::plot_drone_heatmap);
        ImGui::Checkbox("Plot Drone Trace", &testbed::plot_drone_trace);
        ImGui::Checkbox("Plot Drone Speed", &testbed::plot_drone_speed);
      } else if (!plot_graphs) {
        plot_graph1 = false;
        plot_graph2 = false;
        plot_graph3 = false;
        plot_graph4 = false;
        plot_graph5 = false;
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
    ImGui::End();
  }
  void Draw(b2World *world, DebugDraw *debugDraw,
            std::vector<int> foundTreeIDs) {
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
            debugDraw->DrawSegment(v1, v2, b2Color(0.5f, 0.5f, 0.5f));
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
    RegisterTest("Simulator", "Queue Mode", QueueSimulator::Create);
