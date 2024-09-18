//
// Created by Joel Beedle on 17/09/2024.
//
#ifndef UI_BUILDER_H
#define UI_BUILDER_H

#include <memory>
#include <vector>
#include <string>

#include "imgui.h"
#include "ui_component.h"

class UIBuilder {
private:
  std::string windowTitle = "Default Title";
  ImVec2 windowPosition = ImVec2(10.0f, 50.0f);

  std::vector<std::unique_ptr<UIComponent>> components;

public:
  UIBuilder& setTitle(const std::string& title);

  UIBuilder& setPosition(ImVec2 position);

  UIBuilder& addComponent(std::unique_ptr<UIComponent> component);

  void render() const;

  void reset();
};

#define UI_BUILDER_H

#endif //UI_BUILDER_H