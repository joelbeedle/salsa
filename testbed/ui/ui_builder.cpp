//
// Created by Joel Beedle on 17/09/2024.
//

#include "ui_builder.h"

UIBuilder& UIBuilder::setTitle(const std::string& title) {
  windowTitle = title;
  return *this;
}

UIBuilder& UIBuilder::setPosition(ImVec2 position){
  windowPosition = position;
  return *this;
}

// Adds a UI component to the builder
UIBuilder& UIBuilder::addComponent(std::unique_ptr<UIComponent> component) {
  components.push_back(std::move(component));
  return *this;
}

// Renders all components added to the builder
void UIBuilder::render() const {
  // Set the window position and begin rendering the ImGUI window
  ImGui::SetNextWindowPos(windowPosition);
  ImGui::Begin(windowTitle.c_str(), nullptr);

  // Render all added components
  for (auto& component : components) {
    component->render();  // Call render on each UIComponent
  }

  // End the window
  ImGui::End();
}

// Resets the builder by clearing the components
void UIBuilder::reset() {
  components.clear();  // Clear the components vector
}
