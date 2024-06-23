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
  using TargetCreateFunc = std::function<std::shared_ptr<Target>(std::any)>;
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
    registry[name] = [](std::any packedArgs) -> std::shared_ptr<Target> {
      try {
        // Correctly extract and apply the tuple from std::any
        return std::apply(
            [](Args... args) {
              return std::make_shared<T>(std::forward<Args>(args)...);
            },
            std::any_cast<std::tuple<Args...>>(packedArgs));
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
                                              Args&&... args) {
    if (type == "null") {
      return nullptr;
    }
    auto it = registry.find(type);
    if (it != registry.end()) {
      std::any packedArgs = std::make_tuple(std::forward<Args>(args)...);
      return it->second(packedArgs);
    }
    std::cerr << "Target type " << type << " not found in registry."
              << std::endl;
    return nullptr;
  }
};

}  // namespace swarm

#endif  // SWARM_ENTITY_TARGET_FACTORY_H