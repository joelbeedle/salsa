// BehaviourTypes.h
#pragma once

#include <string>

enum class BehaviourType { Flocking, Pheremone, PSO };

struct BehaviourTypeInfo {
  BehaviourType type;
  std::string displayName;
};
