//
// Created by Joel Beedle on 17/09/2024.
//

#ifndef DRONE_CONFIGURATION_COMPONENT_H
#define DRONE_CONFIGURATION_COMPONENT_H
#include "imgui.h"
#include "salsa/core/sim.h"
#include "ui/ui_component.h"

class DroneSettingsComponent : public UIComponent {
private:
  salsa::Sim* sim{};

public:
  explicit DroneSettingsComponent(salsa::Sim* simulation)
      : sim(simulation) {}

  // Render the component
  void render() override {
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);

    if (ImGui::CollapsingHeader("Drone Settings")) {
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
          "droneDetectionRange", &sim->getDroneConfiguration()->droneDetectionRange, 0.0f, 4000.0f);

      // Update drone settings if any of the sliders were changed
      if (droneChanged) {
        sim->updateDroneSettings();
      }
    }
  }
};


#endif //DRONE_CONFIGURATION_COMPONENT_H
