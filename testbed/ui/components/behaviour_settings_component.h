//
// Created by Joel Beedle on 17/09/2024.
//

#ifndef BEHAVIOURSETTINGSCOMPONENT_H
#define BEHAVIOURSETTINGSCOMPONENT_H
#include "imgui.h"
#include "salsa/core/sim.h"
#include "ui/ui_component.h"

class BehaviorSettingsComponent final : public UIComponent {
  salsa::Sim* sim;
public:
  explicit BehaviorSettingsComponent(salsa::Sim* sim) : sim(sim) {}

  void render() override {
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
    }
  }
};

#endif //BEHAVIOURSETTINGSCOMPONENT_H
