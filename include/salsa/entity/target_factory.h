/// @file target_factory.h
/// Contains the `TargetFactory` class, which is used to create `Target`
/// objects.
#ifndef SWARM_ENTITY_TARGET_FACTORY_H
#define SWARM_ENTITY_TARGET_FACTORY_H

#include <any>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <tuple>
#include <typeinfo>

#include "salsa/utils/object_types.h"
#include "target.h"

namespace swarm {

/// @class TargetFactory
/// @brief Factory class for creating Target objects dynamically.
///
/// Allows registration of target types with associated creation functions
/// and facilitates the creation of targets using registered types.
class TargetFactory {
 private:
  using TargetCreateFunc = std::function<std::shared_ptr<Target>(
      b2World*, const b2Vec2&, int, std::any)>;
  static std::map<std::string, TargetCreateFunc> registry;

 public:
  /// Registers a target type with the factory.
  /// @param name The name identifier for the target type.
  /// @param fixedArgs Fixed arguments required for creating the target.
  template <typename T, typename... Args>
  static void registerTarget(const std::string& name, Args... fixedArgs) {
    registry[name] = [fixedArgsTuple = std::make_tuple(fixedArgs...)](
                         b2World* world, const b2Vec2& vec, int id,
                         std::any packedArgs) -> std::shared_ptr<Target> {
      try {
        auto argsTuple =
            std::tuple_cat(std::make_tuple(world, vec, id), fixedArgsTuple);
        return std::apply(
            [](auto&&... args) {
              return std::make_shared<T>(std::forward<decltype(args)>(args)...);
            },
            argsTuple);
      } catch (const std::bad_any_cast& e) {
        std::cerr << "Bad any_cast in factory creation: " << e.what() << '\n';
        return nullptr;
      }
    };
  }
  /// Registers a target type with additional parameters to be unpacked during
  /// creation.
  /// @param name The name identifier for the target type.
  template <typename T, typename... Args>
  static void registerTargetType(const std::string& name) {
    registry[name] = [](b2World* world, const b2Vec2& vec, int id,
                        std::any packedArgs) -> std::shared_ptr<Target> {
      try {
        auto argsTuple =
            std::tuple_cat(std::make_tuple(world, vec, id),
                           std::any_cast<std::tuple<Args...>>(packedArgs));
        return std::apply(
            [](auto&&... args) {
              return std::make_shared<T>(std::forward<decltype(args)>(args)...);
            },
            argsTuple);
      } catch (const std::bad_any_cast& e) {
        std::cerr << "Bad any_cast in factory creation: " << e.what() << '\n';
        return nullptr;
      }
    };
  }

  /// Creates a target of a registered type.
  /// @param type The type identifier of the target.
  /// @param world Pointer to the physics world where the target exists.
  /// @param vec The position in the world where the target will be placed.
  /// @param id The identifier for the target instance.
  /// @param packedArgs Any additional arguments required for creating the
  /// target.
  /// @return A shared_ptr to the created Target, or nullptr if the type is not
  /// registered.
  template <typename... Args>
  static std::shared_ptr<Target> createTarget(const std::string& type,
                                              b2World* world, const b2Vec2& vec,
                                              int id,
                                              const std::any& packedArgs) {
    if (registry.find(type) == registry.end()) {
      std::cerr << "Target type " << type << " not found in registry.\n";
      return nullptr;
    }
    return registry[type](world, vec, id, packedArgs);
  }

  /// Retrieves the names of all registered target types.
  /// @return A vector containing the names of all registered target types.
  static std::vector<std::string> getTargetNames();
};

}  // namespace swarm

#endif  // SWARM_ENTITY_TARGET_FACTORY_H