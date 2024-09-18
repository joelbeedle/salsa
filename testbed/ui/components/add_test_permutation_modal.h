//
// Created by Joel Beedle on 17/09/2024.
//

#ifndef ADD_TEST_PERMUTATION_MODAL_H
#define ADD_TEST_PERMUTATION_MODAL_H
#include <sstream>


#include "base_modal_component.h"
#include "logger.h"
#include "salsa/behaviours/registry.h"
#include "salsa/core/test_queue.h"
#include "salsa/entity/target_factory.h"

class AddTestPermutationModal final : public BaseModalComponent {
    static std::string current_behaviour;
    static std::string current_name;
    static std::string current_map_name;
    static std::string current_listener_name;
    static std::string current_drone_config_name;
    static std::string current_target_name;

    static int new_drone_count;
    static int new_target_count;
    static float new_time_limit;

    static bool to_change;

    static std::unordered_map<std::string, std::string> input_storage;
    static std::vector<std::string> parameter_names;
    static std::vector<int> selections;
    static std::unordered_map<std::string, std::vector<float>> range_storage;

    static std::vector<salsa::TestConfig> base;
    std::unordered_map<std::string, salsa::behaviour::Parameter*> chosen_params;
    static bool generated;

public:
    AddTestPermutationModal(bool& should_open_flag, bool& is_open_flag, bool& pause_flag)
        : BaseModalComponent(should_open_flag, is_open_flag, pause_flag, "Add Test Permutation") {}

protected:
    // Render the modal content
    void renderModalContent() override {
        // Behavior selection
      // Get behaviour name for test
      if (ImGui::BeginCombo("Behaviour", current_name.c_str())) {
        for (const auto behaviourNames = salsa::behaviour::Registry::get().behaviour_names(); auto &name : behaviourNames) {
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
      chosen_params = chosen_behaviour->getParameters();

      if (to_change) {
        input_storage.clear();
        parameter_names.clear();
        selections.clear();
        range_storage.clear();
        for (const auto& [key, value] : chosen_params) {
          parameter_names.push_back(key);
          input_storage[key] =
              std::string(128, '\0');  // Initialize with null characters
          selections.push_back(0);
          range_storage[key] = {0.0f, 0.0f, 0.0f};
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

      const auto listenerNames = salsa::BaseContactListener::getListenerNames();
      current_listener_name =
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
      const auto droneConfigNames =
          salsa::DroneConfiguration::getDroneConfigurationNames();
      current_drone_config_name = droneConfigNames[0];
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

      const auto targetNames = salsa::TargetFactory::getTargetNames();
      current_target_name = targetNames[0];
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
    }

    // Override the default buttons with custom ones
    void renderModalButtons() override;

private:
    static void generatePermutations(std::vector<std::vector<float>> &results,
                                     const std::vector<std::vector<float>> &lists,
                                     std::vector<float> current = {}, size_t depth = 0, size_t maxDepth = 1000);

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
      if (step == 0.0f) {
        throw std::invalid_argument("Step size cannot be zero, which would cause an infinite loop.");
      }

      if (min > max && step > 0) {
        throw std::invalid_argument("Step size must be negative for a decreasing range (min > max).");
      }

      if (min < max && step < 0) {
        throw std::invalid_argument("Step size must be positive for an increasing range (min < max).");
      }

    std::vector<float> range;
    for (float value = min; value <= max + step / 2; value += step) {
      value = static_cast<float>(std::round(value * 1e6) / 1e6);
      if (value <= max) {
        range.push_back(value);
      }
    }
    return range;
  }
};

#endif //ADD_TEST_PERMUTATION_MODAL_H
