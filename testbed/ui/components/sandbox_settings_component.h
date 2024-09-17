//
// Created by Joel Beedle on 17/09/2024.
//

#ifndef SANDBOX_SETTINGS_COMPONENT_H
#define SANDBOX_SETTINGS_COMPONENT_H
#include "draw.h"
#include "imgui.h"
#include "salsa/core/sim.h"
#include "ui/ui_component.h"

class SimulationSettingsComponent : public UIComponent {
private:
    salsa::Sim* sim;     // Reference to the simulation object
    bool& pause;                // Reference to the pause flag
    bool& update_drone_count_;  // Reference to the drone count update flag
    int& new_count;             // Reference to the new drone count
    b2World*& m_world;          // Reference to the world object
    Camera& g_camera;           // Reference to the camera

public:
    SimulationSettingsComponent(salsa::Sim* simulation, bool& pause_flag, bool& update_drone_count_flag,
                                int& drone_count, b2World*& world, Camera& camera)
        : sim(simulation), pause(pause_flag), update_drone_count_(update_drone_count_flag),
          new_count(drone_count), m_world(world), g_camera(camera) {}

    // Override the render method
    void render() override {
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::CollapsingHeader("Simulation Settings")) {
            ImGui::SeparatorText("Simulation Settings");

            // Change drone count
            static int changed_count = new_count;
            if (bool changed = ImGui::SliderInt("Drone Count", &new_count, 0, 100)) {
                changed_count = new_count;
                pause = true;
                update_drone_count_ = true;
            }

            // Map selection
            const auto mapNames = salsa::map::getMapNames();
            static std::string current_map_name = mapNames[0];
            if (ImGui::BeginCombo("Map", current_map_name.c_str())) {
                for (auto& name : mapNames) {
                    bool isSelected = (current_map_name == name);
                    if (ImGui::Selectable(name.c_str(), isSelected)) {
                        // Change the current map in the simulation and reset
                        current_map_name = name;
                        sim->changeMap(name);
                        m_world = sim->getWorld();
                        g_camera.m_center = sim->getDroneSpawnPosition();
                        g_camera.m_zoom = 10.0f;  // Reset camera zoom
                    }
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            auto new_map = salsa::map::getMap(current_map_name);
        }
    }
};

#endif //SANDBOX_SETTINGS_COMPONENT_H
