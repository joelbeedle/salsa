//
// Created by Joel Beedle on 17/09/2024.
//

#ifndef UI_COMPONENT_H
#define UI_COMPONENT_H

class UIComponent {
public:
  virtual void render() = 0;  // Pure virtual function to render UI component
  virtual ~UIComponent() = default;  // Virtual destructor
};

#endif //UI_COMPONENT_H
