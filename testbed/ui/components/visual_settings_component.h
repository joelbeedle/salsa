//
// Created by Joel Beedle on 17/09/2024.
//

#ifndef VISUAL_SETTINGS_COMPONENT_H
#define VISUAL_SETTINGS_COMPONENT_H
#include "imgui.h"
#include "salsa/core/sim.h"
#include "ui/ui_component.h"

class VisualSettingsComponent : public UIComponent {
private:
  bool& draw_visual_range_;  // Reference to the visual range drawing state
  bool& draw_targets_;       // Reference to the target drawing state
  salsa::Sim* sim{};    // Reference to the simulation for resetting

public:
  VisualSettingsComponent(bool& draw_visual_range, bool& draw_targets, salsa::Sim* sim)
      : draw_visual_range_(draw_visual_range), draw_targets_(draw_targets), sim(sim) {}

  // Render the component
  void render() override {
    // Separator for visual settings
    ImGui::SeparatorText("Visual Settings");

    // Checkbox to toggle drawing the drone visual range
    ImGui::Checkbox("Draw Drone visual range", &draw_visual_range_);

    // Checkbox to toggle drawing the targets
    ImGui::Checkbox("Draw Targets", &draw_targets_);

    // Button to reset the simulation
    if (ImGui::Button("Reset Simulation")) {
      sim->reset();  // Reset the simulation when the button is clicked
    }
  }
};

#endif //VISUAL_SETTINGS_COMPONENT_H
