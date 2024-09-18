//
// Created by Joel Beedle on 17/09/2024.
//

#ifndef ADD_TEST_PERMUTATION_MODAL_H
#define ADD_TEST_PERMUTATION_MODAL_H
#include <sstream>


#include "base_modal_component.h"
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
        const auto behaviourNames = salsa::behaviour::Registry::get().behaviour_names();
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
      chosen_params = chosen_behaviour->getParameters();

      if (to_change) {
        input_storage.clear();
        parameter_names.clear();
        selections.clear();
        range_storage.clear();
        for (const auto& key : chosen_params | std::views::keys) {
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
    void renderModalButtons() override {
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
        for (const auto& config : base) {
          salsa::TestQueue::push(config);
        }
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
          for (const auto &config : salsa::TestQueue::getTests()) {
            old_tests.push_back(config);
          }
          salsa::TestQueue::getTests().clear();
          for (const auto &config : base) {
            salsa::TestQueue::push(config);
          }
          salsa::TestQueue::save(filename);
          salsa::TestQueue::getTests().clear();
          for (const auto &config : old_tests) {
            salsa::TestQueue::push(config);
          }
          ImGui::CloseCurrentPopup();
        }

        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
          ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
      }

      ImGui::SameLine();
      if (ImGui::Button("Cancel", ImVec2(120, 0))) {
        ImGui::CloseCurrentPopup();
      }

    }


private:
    // Generate permutations logic
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
};

#endif //ADD_TEST_PERMUTATION_MODAL_H
