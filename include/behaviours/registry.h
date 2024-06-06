// Registry.h
#ifndef SWARM_SIM_UTILS_BEHAVIOUR_REGISTRY_H
#define SWARM_SIM_UTILS_BEHAVIOUR_REGISTRY_H

#include <memory>
#include <unordered_map>
#include <vector>

#include "behaviours/behaviour.h"

namespace swarm_sim {
namespace behaviour {
class Registry {
 private:
  std::unordered_map<std::string, std::unique_ptr<Behaviour>> Behaviours;

 public:
  static Registry& getInstance() {
    static Registry instance;
    return instance;
  }

  void add(const std::string& name, std::unique_ptr<Behaviour> Behaviour) {
    Behaviours[name] = std::move(Behaviour);
  }

  Behaviour* getBehaviour(const std::string& name) {
    if (Behaviours.find(name) != Behaviours.end()) {
      return Behaviours[name].get();
    }
    return nullptr;
  }

  std::vector<std::string> getBehaviourNames() const {
    std::vector<std::string> names;
    for (const auto& pair : Behaviours) {
      names.push_back(pair.first);
    }
    return names;
  }

  // Prevent copy & assignment
  Registry(const Registry&) = delete;
  Registry& operator=(const Registry&) = delete;

 private:
  Registry() {}
};

}  // namespace behaviour
}  // namespace swarm_sim

#endif  // SWARM_SIM_UTILS_BEHAVIOUR_REGISTRY_H