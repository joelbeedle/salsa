//
// Created by Joel Beedle on 17/09/2024.
//

#ifndef ADD_TEST_MODAL_H
#define ADD_TEST_MODAL_H
#include <string>


#include "base_modal_component.h"
#include "imgui.h"
#include "salsa/behaviours/registry.h"
#include "salsa/core/map.h"
#include "salsa/core/test_queue.h"
#include "salsa/entity/drone_configuration.h"
#include "salsa/entity/target_factory.h"
#include "salsa/utils/base_contact_listener.h"

class AddTestModal final : public BaseModalComponent {
  // Add Test-specific state (these can be customized as needed)
  static std::string current_behaviour;
  static int drone_count;
  static int target_count;
  static float new_time_limit;
  static b2World* world;
  static std::string current_target_name;
  static std::string current_map_name;
  static std::string current_name;
  static float time_limit;
  static bool to_change;
  static std::unordered_map<std::string, salsa::behaviour::Parameter *>
      new_params;
  static std::string current_drone_config_name;
  static std::string current_listener_name;
public:
  AddTestModal(bool& should_open_flag, bool& is_open_flag, bool& pause_flag)
      : BaseModalComponent(should_open_flag, is_open_flag, pause_flag, "Add Test") {}

protected:
  // Render the specific content for the "Add Test" modal
  void renderModalContent() override {
    if (ImGui::BeginCombo("Behaviour", current_name.c_str())) {
      const auto behaviourNames =
            salsa::behaviour::Registry::get().behaviour_names();

        for (auto &name : behaviourNames) {
          const bool isSelected = (current_name == name);
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
      const auto chosen_behaviour =
          salsa::behaviour::Registry::get().behaviour(current_name);
      auto chosen_params = chosen_behaviour->getParameters();
      if (new_params.empty() || to_change) {
        new_params.clear();
        for (const auto & [fst, snd] : chosen_params) {
          new_params[fst] =
              snd->clone();  // Clone each Parameter and insert into the
          // new map
          to_change = false;
        }
      }
      for (auto [name, parameter] : new_params) {
        ImGui::SliderFloat(name.c_str(), &(parameter->value()),
                           parameter->min_value(), parameter->max_value());
      }

      const auto mapNames = salsa::map::getMapNames();
      if (ImGui::BeginCombo("Map", current_map_name.c_str())) {
        for (auto &name : mapNames) {
          const bool isSelected = (current_map_name == name);
          if (ImGui::Selectable(name.c_str(), isSelected)) {
            current_map_name = name;
          }
          if (isSelected) {
            ImGui::SetItemDefaultFocus();
          }
        }
        ImGui::EndCombo();
      }

      const auto listenerNames = salsa::BaseContactListener::getListenerNames();
      current_listener_name =
          listenerNames.empty() ? "" : listenerNames[0];

      if (ImGui::BeginCombo("Listener Type", current_listener_name.c_str())) {
        for (auto &name : listenerNames) {
          const bool isSelected = (current_listener_name == name);
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
          const bool isSelected = (current_drone_config_name == name);
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
          const bool isSelected = (current_target_name == name);
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
      ImGui::InputInt("Drone Count", &drone_count);
      // Set number of targets
      ImGui::InputInt("Target Count", &target_count);
      // Set time limit
      ImGui::InputFloat("Time Limit", &time_limit);
  }

  // Define what happens when "Add" is pressed
  void onAdd() override {
    const salsa::TestConfig new_config = {
      current_name,
      salsa::Behaviour::convertParametersToFloat(new_params),
      current_drone_config_name,
      current_map_name,
      drone_count,
      target_count,
      time_limit,
      current_target_name,
      current_listener_name};
    salsa::TestQueue::push(new_config);
  }
};



#endif //ADD_TEST_MODAL_H
