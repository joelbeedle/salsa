#ifndef SWARM_ENTITY_TARGET_FACTORY_H
#define SWARM_ENTITY_TARGET_FACTORY_H

#include <cxxabi.h>

#include <any>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <tuple>
#include <typeinfo>

#include "entity/target.h"

namespace swarm {

class TargetFactory {
 private:
  using TargetCreateFunc = std::function<std::shared_ptr<Target>(
      b2World*, const b2Vec2&, int, std::any)>;
  static std::map<std::string, TargetCreateFunc> registry;

  static std::string mine(const char* name) {
    int status = -1;
    std::unique_ptr<char, void (*)(void*)> res{
        abi::__cxa_demangle(name, NULL, NULL, &status), std::free};
    return (status == 0) ? res.get() : name;
  }

 public:
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
                  << mine(typeid(std::tuple<Args...>).name()) << '\n';
        std::cerr << "Received type: " << mine(packedArgs.type().name())
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
      return nullptr;
    }
    auto it = registry.find(type);
    if (it != registry.end()) {
      return it->second(world, vec, id, packedArgs);
    }
    std::cerr << "Target type " << type << " not found in registry."
              << std::endl;
    return nullptr;
  }
};

}  // namespace swarm

#endif  // SWARM_ENTITY_TARGET_FACTORY_H