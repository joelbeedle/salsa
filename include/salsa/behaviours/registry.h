/// @file registry.h
/// @brief Contains the `Registry` class, for managing `Behaviour` instances.
#ifndef SWARM_SIM_UTILS_BEHAVIOUR_REGISTRY_H
#define SWARM_SIM_UTILS_BEHAVIOUR_REGISTRY_H

#include <memory>
#include <unordered_map>
#include <vector>

#include "salsa/behaviours/behaviour.h"

namespace salsa {
namespace behaviour {
/// @brief Singleton class that manages and provides access to Behaviour
/// instances.
///
/// This class uses the Singleton design pattern to ensure that there
/// is only one instance of the Registry throughout the application. It provides
/// methods to add, retrieve, and list behaviours dynamically at runtime.
class Registry {
 private:
  std::unordered_map<std::string, std::unique_ptr<Behaviour>>
      behaviours_;  ///< Stores behaviours keyed by their names.

 public:
  /// @brief Returns the singleton instance of the Registry.
  /// @return Reference to the singleton instance of Registry.
  static Registry &get() {
    static Registry instance;
    return instance;
  }

  /// @brief Adds a Behaviour instance to the registry.
  ///
  /// @param name The name key under which the Behaviour will be stored.
  /// @param Behaviour Pointer to the Behaviour instance to be stored.
  bool add(const std::string &name, std::unique_ptr<Behaviour> Behaviour) {
    behaviours_[name] = std::move(Behaviour);
    return true;
  }

  /// @brief Retrieves a Behaviour by name.
  ///
  /// @param name The name key of the Behaviour to retrieve.
  /// @return Pointer to the Behaviour, or nullptr if not found.
  Behaviour *behaviour(const std::string &name) {
    if (behaviours_.find(name) != behaviours_.end()) {
      return behaviours_[name].get();
    }
    return nullptr;
  }

  /// @brief Returns a list of all Behaviour names stored in the Registry.
  ///
  /// @return Vector of strings containing the names of all registered
  /// behaviours_.
  std::vector<std::string> behaviour_names() const {
    std::vector<std::string> names;
    for (const auto &pair : behaviours_) {
      names.push_back(pair.first);
    }
    return names;
  }

  /// @brief Removes a Behaviour from the Registry.
  /// @param name The name of the Behaviour to remove.
  /// @return True if the Behaviour was removed, false if it was not found.
  bool remove(const std::string &name) { return behaviours_.erase(name) > 0; }

  /// @brief Prevent copy construction
  /// @param rhs The Registry instance intended to copy from
  Registry(const Registry &) = delete;

  /// @brief Prevent assignment
  /// @param rhs The Registry instance intended to assign from
  /// @return deletes the assigned value
  Registry &operator=(const Registry &) = delete;

 private:
  /// @brief Private constructor to enforce singleton pattern.
  Registry() {}
};

}  // namespace behaviour
}  // namespace salsa

#endif  // SWARM_SIM_UTILS_BEHAVIOUR_REGISTRY_H