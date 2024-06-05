// BehaviourRegistry.h
#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "swarm_behaviour.h"

class SwarmBehaviourRegistry {
 private:
  std::unordered_map<std::string, std::unique_ptr<SwarmBehaviour>>
      SwarmBehaviours;

 public:
  static SwarmBehaviourRegistry& getInstance() {
    static SwarmBehaviourRegistry instance;
    return instance;
  }

  void add(const std::string& name,
           std::unique_ptr<SwarmBehaviour> SwarmBehaviour) {
    SwarmBehaviours[name] = std::move(SwarmBehaviour);
  }

  SwarmBehaviour* getSwarmBehaviour(const std::string& name) {
    if (SwarmBehaviours.find(name) != SwarmBehaviours.end()) {
      return SwarmBehaviours[name].get();
    }
    return nullptr;
  }

  std::vector<std::string> getSwarmBehaviourNames() const {
    std::vector<std::string> names;
    for (const auto& pair : SwarmBehaviours) {
      names.push_back(pair.first);
    }
    return names;
  }

  // Prevent copy & assignment
  SwarmBehaviourRegistry(const SwarmBehaviourRegistry&) = delete;
  SwarmBehaviourRegistry& operator=(const SwarmBehaviourRegistry&) = delete;

 private:
  SwarmBehaviourRegistry() {}
};
