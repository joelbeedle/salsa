//
// Created by Joel Beedle on 17/09/2024.
//

#ifndef TEST_QUEUE_COMPONENT_H
#define TEST_QUEUE_COMPONENT_H
#include <exception>


#include "imgui.h"
#include "salsa/core/test_queue.h"
#include "ui/ui_component.h"


class TestQueueComponent final : public UIComponent {
private:
    salsa::Sim* sim{};  // Reference to simulation for fetching current test config
    bool& pause;             // Reference to the pause state
    bool& next_frame;        // Reference to the state for moving to the next frame
    bool& skipped_test;      // Reference for skipping the current test
    bool& add_new_test_;     // Reference to add new test flag
    bool& add_test_permutation_; // Reference for adding a new test permutation
    bool added_new_test_ = false; // Track if a new test was added
    bool added_test_permutation_ = false; // Track if test permutation was added

    // Modal flags
    bool show_add_test_modal = false;
    bool show_add_test_permutation_modal = false;

    // AddTestModal and AddTestPermutationModal instances
    AddTestModal* add_test_modal = nullptr;
    AddTestPermutationModal* add_test_permutation_modal = nullptr;

public:
    TestQueueComponent(salsa::Sim* simulation, bool& pause_flag, bool& next_frame_flag, bool& skipped_test_flag,
                       bool& add_new_test_flag, bool& add_test_permutation_flag)
        : sim(simulation), pause(pause_flag), next_frame(next_frame_flag), skipped_test(skipped_test_flag),
          add_new_test_(add_new_test_flag), add_test_permutation_(add_test_permutation_flag) {

        // Initialize the modals with the appropriate flags
        add_test_modal = new AddTestModal(show_add_test_modal, added_new_test_, pause);
        add_test_permutation_modal = new AddTestPermutationModal(show_add_test_permutation_modal, added_test_permutation_, pause);
    }

    ~TestQueueComponent() {
        // Clean up dynamically allocated modals
        delete add_test_modal;
        delete add_test_permutation_modal;
    }

    // Render the component
    void render() override {
        ImGui::SeparatorText("Simulation Queue");

        // Display current test
        ImGui::BeginChild("Current Test", ImVec2(200, 100), true, ImGuiWindowFlags_None | ImGuiWindowFlags_MenuBar);
        if (ImGui::BeginMenuBar()) {
            ImGui::MenuItem("Current Test", nullptr, false, false);
        }
        ImGui::EndMenuBar();

        salsa::TestConfig current_config = sim->test_config();
        ImGui::Text("Behaviour: %s", current_config.behaviour_name.c_str());
        ImGui::Text("Drone Count: %d", current_config.num_drones);
        ImGui::Text("Target Count: %d", current_config.num_targets);
        ImGui::Text("Time Limit: %f", current_config.time_limit);
        ImGui::EndChild();

        ImGui::SameLine();

        // Display next test in queue
        ImGui::BeginChild("Next Test", ImVec2(200, 100), true, ImGuiWindowFlags_None | ImGuiWindowFlags_MenuBar);
        if (ImGui::BeginMenuBar()) {
            ImGui::MenuItem("Next Test", nullptr, false, false);
        }
        ImGui::EndMenuBar();

        try {
            salsa::TestConfig next_config = salsa::TestQueue::peek();
            ImGui::Text("Behaviour: %s", next_config.behaviour_name.c_str());
            ImGui::Text("Drone Count: %d", next_config.num_drones);
            ImGui::Text("Target Count: %d", next_config.num_targets);
            ImGui::Text("Time Limit: %f", next_config.time_limit);
        } catch (const std::exception&) {
            ImGui::Text("No more tests in queue");
        }
        ImGui::EndChild();

        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 5.0f);

        // Display the test queue
        ImGui::BeginChild("Test Queue", ImVec2(0, 100), true, ImGuiWindowFlags_None | ImGuiWindowFlags_MenuBar);
        if (ImGui::BeginMenuBar()) {
            ImGui::MenuItem("Test Queue", nullptr, false, false);
            if (ImGui::BeginMenu("Options")) {
                if (ImGui::MenuItem("Clear Queue")) {
                    salsa::TestQueue::getTests().clear();
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("+")) {
                if (ImGui::MenuItem("New Test")) {
                    show_add_test_modal = true;  // Trigger modal
                    pause = true;
                }
                if (ImGui::MenuItem("New Test Permutation")) {
                    show_add_test_permutation_modal = true;  // Trigger modal
                    pause = true;
                }
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }

        // Loop through and display the test queue
        auto& tests = salsa::TestQueue::getTests();
        for (int i = 0; i < tests.size(); ++i) {
            auto& test = tests[i];
            if (ImGui::CollapsingHeader(test.behaviour_name.c_str(), &test.keep)) {
                ImGui::Text("Behaviour: %s", test.behaviour_name.c_str());
                ImGui::Text("Drone Count: %d", test.num_drones);
                ImGui::Text("Target Count: %d", test.num_targets);
                ImGui::Text("Time Limit: %f", test.time_limit);
            }

            // Drag-and-drop to reorder the queue
            if (ImGui::IsItemActive() && !ImGui::IsItemHovered()) {
                int next_index = i + (ImGui::GetMouseDragDelta(0).y < 0.f ? -1 : 1);
                if (next_index >= 0 && next_index < tests.size()) {
                    std::swap(tests[i], tests[next_index]);
                    ImGui::ResetMouseDragDelta();
                }
            }

            // Remove the test from the queue if requested
            if (!test.keep) {
                tests.erase(tests.begin() + i);
                i--;  // Adjust loop index to account for the removed element
            }
        }
        ImGui::EndChild();
        ImGui::PopStyleVar();

        // Control buttons for next test and saving the queue
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
                salsa::TestQueue::save(filename);
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        // Render modals
        add_test_modal->render();            // Render Add Test Modal
        add_test_permutation_modal->render(); // Render Add Test Permutation Modal

        // Reset modal flags
        if (added_new_test_) {
            pause = false;
            added_new_test_ = false;
        }
        if (added_test_permutation_) {
            pause = false;
            added_test_permutation_ = false;
        }
    }
};


#endif //TEST_QUEUE_COMPONENT_H
