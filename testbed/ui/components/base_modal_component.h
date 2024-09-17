//
// Created by Joel Beedle on 17/09/2024.
//

#ifndef BASE_MODAL_COMPONENT_H
#define BASE_MODAL_COMPONENT_H
#include <string>


#include "imgui.h"
#include "ui/ui_component.h"

class BaseModalComponent : public UIComponent {
protected:
    bool& should_open;      // Indicates whether the modal should open
    bool& is_open;          // Indicates whether the modal is currently open
    bool& pause;            // Pause flag to pause background processes when modal is open
    const std::string modal_title;  // The title of the modal

public:
    BaseModalComponent(bool& should_open_flag, bool& is_open_flag, bool& pause_flag, const std::string& title)
        : should_open(should_open_flag), is_open(is_open_flag), pause(pause_flag), modal_title(title) {}

    // Render function manages modal opening/closing lifecycle
    void render() override {
        if (should_open) {
            pause = true;
            ImGui::OpenPopup(modal_title.c_str());
            should_open = false;
        }

        if (is_open) {
            pause = false;
            is_open = false;
        }

        const ImGuiIO& io = ImGui::GetIO();
        ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

        if (ImGui::BeginPopupModal(modal_title.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            renderModalContent();  // Call the derived class to render modal-specific content
            renderModalButtons();  // Render the common buttons like "Add" and "Cancel"
            ImGui::EndPopup();
        }
    }

protected:
    // Derived classes should implement this to define the modal-specific content
    virtual void renderModalContent() = 0;

    // Common buttons like "Add" and "Cancel", but derived classes can override for custom behavior
    virtual void renderModalButtons() {
        if (ImGui::Button("Add", ImVec2(120, 0))) {
            onAdd();  // Derived class defines what happens on "Add"
            is_open = true;
            ImGui::CloseCurrentPopup();
        }

        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) {
            is_open = true;
            ImGui::CloseCurrentPopup();
        }
    }

    // Derived classes can override this to define "Add" button behavior
    virtual void onAdd() {};
};



#endif //BASE_MODAL_COMPONENT_H
