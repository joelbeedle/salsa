#ifndef SWARM_ENTITY_TARGET_FACTORY_H
#define SWARM_ENTITY_TARGET_FACTORY_H

#include <any>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <tuple>
#include <typeinfo>

#include "entity/target.h"
#include "utils/object_types.h"

namespace swarm {

class TargetFactory {
 private:
  using TargetCreateFunc = std::function<std::shared_ptr<Target>(
      b2World*, const b2Vec2&, int, std::any)>;
  static std::map<std::string, TargetCreateFunc> registry;

 public:
  template <typename T, typename... Args>
  static void registerTarget(const std::string& name, Args... fixedArgs) {
    registry[name] = [fixedArgsTuple = std::make_tuple(fixedArgs...)](
                         b2World* world, const b2Vec2& vec, int id,
                         std::any packedArgs) -> std::shared_ptr<Target> {
      try {
        // Extract the additional tuple from std::any, then concatenate with
        // registered and common parameters
        auto argsTuple =
            std::tuple_cat(std::make_tuple(world, vec, id), fixedArgsTuple);

        // Use std::apply to construct the object with unpacked arguments
        return std::apply(
            [](auto&&... args) {
              return std::make_shared<T>(std::forward<decltype(args)>(args)...);
            },
            argsTuple);
      } catch (const std::bad_any_cast& e) {
        std::cerr << "Bad any_cast in factory creation: " << e.what() << '\n';
        std::cerr << "Expected types: " << typeid(std::tuple<Args...>).name()
                  << '\n';  // You might still need demangling here if required
        std::cerr << "Received type: " << typeid(packedArgs).name() << '\n';
        return nullptr;
      }
    };
  }

  template <typename T, typename... Args>
  static void registerTargetType(const std::string& name) {
    registry[name] = [](b2World* world, const b2Vec2& vec, int id,
                        std::any packedArgs) -> std::shared_ptr<Target> {
      try {
        // Extract the tuple from std::any, then concatenate with common
        // parameters
        auto argsTuple =
            std::tuple_cat(std::make_tuple(world, vec, id),
                           std::any_cast<std::tuple<Args...>>(packedArgs));

        // Use std::apply to construct the object with unpacked arguments
        return std::apply(
            [](auto&&... args) {
              return std::make_shared<T>(std::forward<decltype(args)>(args)...);
            },
            argsTuple);
      } catch (const std::bad_any_cast& e) {
        std::cerr << "Bad any_cast in factory creation: " << e.what() << '\n';
        std::cerr << "Expected types: "
                  << type(typeid(std::tuple<Args...>).name()) << '\n';
        std::cerr << "Received type: " << type(packedArgs.type().name())
                  << '\n';
        return nullptr;
      }
    };
  }

  template <typename... Args>
  static std::shared_ptr<Target> createTarget(const std::string& type,
                                              b2World* world, const b2Vec2& vec,
                                              int id,
                                              const std::any& packedArgs) {
    if (type == "null") {
      std::cout << "Target type is null." << std::endl;
      return nullptr;
    }
    auto it = registry.find(type);
    if (it != registry.end()) {
      return it->second(world, vec, id, packedArgs);
    }
    std::cout << "Target type " << type << " not found in registry."
              << std::endl;
    return nullptr;
  }

  static std::vector<std::string> getTargetNames() {
    std::vector<std::string> names;
    for (const auto& [name, _] : registry) {
      names.push_back(name);
    }
    return names;
  }
};

}  // namespace swarm

#endif  // SWARM_ENTITY_TARGET_FACTORY_H