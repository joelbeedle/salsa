// BehaviourFactory.h
#pragma once
#include <variant>

#include "FlockingBehaviour.h"
#include "PSOBehaviour.h"
#include "PheremoneBehaviour.h"
#include "SwarmBehaviour.h"

class BehaviourFactory {
 public:
  using BehaviourParams =
      std::variant<FlockingParameters, PheremoneParameters, PSOParameters>;

  static SwarmBehaviour *createBehaviour(BehaviourType type,
                                         BehaviourParams params) {
    switch (type) {
      case BehaviourType::Flocking:
        return new FlockingBehaviour(
            extractParameters<FlockingParameters>(params));
      case BehaviourType::Pheremone:
        return new PheremoneBehaviour(
            extractParameters<PheremoneParameters>(params));
      case BehaviourType::PSO:
        return new PSOBehaviour(extractParameters<PSOParameters>(params));
      default:
        throw std::runtime_error("Unknown behaviour type");
    }
  }

 private:
  template <typename T>
  static T extractParameters(const BehaviourParams &params) {
    if (auto p = std::get_if<T>(&params)) {
      return *p;
    } else {
      throw std::runtime_error("Incorrect parameters for behaviour");
    }
  }
};
