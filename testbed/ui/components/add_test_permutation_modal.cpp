//
// Created by Joel Beedle on 17/09/2024.
//

#include "add_test_permutation_modal.h"
const auto behaviourNames = salsa::behaviour::Registry::get().behaviour_names();
const auto mapNames = salsa::map::getMapNames();

std::string AddTestPermutationModal::current_behaviour;
std::string AddTestPermutationModal::current_name = behaviourNames[0];
std::string AddTestPermutationModal::current_listener_name;
std::string AddTestPermutationModal::current_drone_config_name;
std::string AddTestPermutationModal::current_target_name;
std::string AddTestPermutationModal::current_map_name = mapNames[0];
int AddTestPermutationModal::new_drone_count = 0;
int AddTestPermutationModal::new_target_count = 0;
float AddTestPermutationModal::new_time_limit = 0.0f;

bool AddTestPermutationModal::to_change = false;

std::unordered_map<std::string, std::string> AddTestPermutationModal::input_storage;
std::vector<std::string> AddTestPermutationModal::parameter_names;
std::vector<int> AddTestPermutationModal::selections;
std::unordered_map<std::string, std::vector<float>> AddTestPermutationModal::range_storage;
bool AddTestPermutationModal::generated = false;
std::vector<salsa::TestConfig> AddTestPermutationModal::base;

void AddTestPermutationModal::renderModalButtons() {
  if (ImGui::Button("Generate")) {
    base.clear();
    std::vector<std::vector<float>> lists;

    // Parse parameters
    for (size_t i = 0; i < parameter_names.size(); ++i) {
      try {
        const auto &name = parameter_names[i];
        if (selections[i] == 0) {  // Range
          std::istringstream iss(input_storage[name]);
          float min, max, step;
          iss >> min >> max >> step;
          try {
            lists.push_back(generateRange(min, max, step));
          } catch (const std::exception &e) {
            throw std::runtime_error("Error generating range for parameter '" + name + "': " + std::string(e.what()));
          }
        } else {  // List
          lists.push_back(parseList(input_storage[name]));
        }
      } catch (const std::exception &e) {
        LOG_ERROR("Error parsing parameters: {}", e.what());
      }
    }

    // Generate permutations
    std::vector<std::vector<float>> permutations;
    LOG_INFO("Generating {}", lists.size());

    try {
      generatePermutations(permutations, lists);
    } catch (const std::exception &e) {
      LOG_ERROR("Failed to generate permutations, {}", e.what());
      std::cout << "Failed to generate permutations, {}" << std::endl;
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

void AddTestPermutationModal::generatePermutations(std::vector<std::vector<float>> &results,
                                 const std::vector<std::vector<float>> &lists,
                                 std::vector<float> current, const size_t depth, const size_t maxDepth) {
  // Check if the recursion depth exceeds the limit
  if (depth > maxDepth) {
    throw std::runtime_error("Recursion depth limit exceeded. Potential infinite recursion.");
  }

  // Base case: if we've reached the end of the lists, store the current permutation
  if (depth == lists.size()) {
    results.push_back(current);
    return;
  }

  // Recursive case: iterate over the current depth's list
  for (const auto &item : lists[depth]) {
    current.push_back(item);
    generatePermutations(results, lists, current, depth + 1, maxDepth);  // Pass maxDepth
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

