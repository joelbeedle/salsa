// BehaviourTypes.h
#pragma once

#include <string>

enum class BehaviourType { Flocking, Pheremone };

struct BehaviourTypeInfo {
  BehaviourType type;
  std::string displayName;
};
