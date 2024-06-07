/// @file registry.h
/// @brief Contains the `Registry` class, for managing `Behaviour` instances.
#ifndef SWARM_SIM_UTILS_BEHAVIOUR_REGISTRY_H
#define SWARM_SIM_UTILS_BEHAVIOUR_REGISTRY_H

#include <memory>
#include <unordered_map>
#include <vector>

#include "behaviours/behaviour.h"

namespace swarm {
namespace behaviour {
/// @brief Singleton class that manages and provides access to Behaviour
/// instances.
/// @details This class uses the Singleton design pattern to ensure that there
/// is only one instance of the Registry throughout the application. It provides
/// methods to add, retrieve, and list behaviours dynamically at runtime.
class Registry {
 private:
  std::unordered_map<std::string, std::unique_ptr<Behaviour>>
      Behaviours;  ///< Stores behaviours keyed by their names.

 public:
  /// @brief Returns the singleton instance of the Registry.
  /// @return Reference to the singleton instance of Registry.
  static Registry& getInstance() {
    static Registry instance;
    return instance;
  }

  /// @brief Adds a Behaviour instance to the registry.
  /// @param name The name key under which the Behaviour will be stored.
  /// @param Behaviour Pointer to the Behaviour instance to be stored.
  void add(const std::string& name, std::unique_ptr<Behaviour> Behaviour) {
    Behaviours[name] = std::move(Behaviour);
  }

  /// @brief Retrieves a Behaviour by name.
  /// @param name The name key of the Behaviour to retrieve.
  /// @return Pointer to the Behaviour, or nullptr if not found.
  Behaviour* getBehaviour(const std::string& name) {
    if (Behaviours.find(name) != Behaviours.end()) {
      return Behaviours[name].get();
    }
    return nullptr;
  }

  /// @brief Returns a list of all Behaviour names stored in the Registry.
  /// @return Vector of strings containing the names of all registered
  /// Behaviours.
  std::vector<std::string> getBehaviourNames() const {
    std::vector<std::string> names;
    for (const auto& pair : Behaviours) {
      names.push_back(pair.first);
    }
    return names;
  }

  /// @brief Prevent copy construction
  /// @param rhs The Registry instance intended to copy from
  Registry(const Registry&) = delete;

  /// @brief Prevent assignment
  /// @param rhs The Registry instance intended to assign from
  /// @return deletes the assigned value
  Registry& operator=(const Registry&) = delete;

 private:
  /// @brief Private constructor to enforce singleton pattern.
  Registry() {}
};

}  // namespace behaviour
}  // namespace swarm

#endif  // SWARM_SIM_UTILS_BEHAVIOUR_REGISTRY_H