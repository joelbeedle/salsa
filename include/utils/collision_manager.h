/// @file collision_manager.h
/// @brief Handles collision configurations and updates for various types
/// deriving from `Entity`.
#ifndef SWARM_UTILS_COLLISION_MANAGER_H
#define SWARM_UTILS_COLLISION_MANAGER_H

#include <box2d/box2d.h>

#ifdef __GNUG__
#include <cxxabi.h>
#endif  // __GNUG__
#include <bitset>
#include <cstdint>
#include <iostream>
#include <map>
#include <typeindex>
#include <vector>

#include "spdlog/spdlog.h"

namespace swarm {

/// @brief Collision configuration for a given entity type.
///
/// Category bits are used to define the category of an entity. At most
/// there can be 16 different categories. Mask bits are used to define the
/// categories that an entity can collide with. Example category bits are
/// `0b0001`, `0b0010`, `0b0100`, `0b1000`. Example mask bits could be `0b0011`,
/// for an entity that collides with category bits `0b0001` and `0b0010`.
struct CollisionConfig {
  uint16_t categoryBits;  ///< The category bit of an entity
  uint16_t maskBits;  ///< The other category bits that an entity collides with.
};

/// @brief Manages collision configurations for different object types.
///
/// CollisionManager uses singleton pattern to maintain a consistent
/// state across the system. It offers functionality to register new types with
/// collision partners and update collision masks dynamically based on
/// registered types and their partners.
class CollisionManager {
 private:
  /// @brief Tracks the next available category bit for registering new types.
  static uint16_t next_category_bit_;
  template <typename T>
  struct TypeKey {
    static inline const char* value() { return typeid(T).name(); }
  };
  /// @brief Stores collision configuration for each registered type.
  static std::unordered_map<std::string, CollisionConfig,
                            std::hash<std::string>, std::equal_to<>>
      configurations_;
  /// @brief Maps each type to its collision partners.
  static std::unordered_map<std::string, std::vector<std::string>,
                            std::hash<std::string>, std::equal_to<>>
      collision_partners_;

  /// @brief Updates the mask bits for all registered types based on their
  /// collision partners.
  ///
  /// This method iterates through each registered type, computes the mask bits
  /// by combining the category bits of its collision partners, and updates the
  /// mask bits accordingly.
  static void updateMaskBits();

#ifdef __GNUG__
  static std::string internal_demangle(std::string name) {
    int status = -1;
    std::unique_ptr<char, void (*)(void*)> res{
        abi::__cxa_demangle(name.c_str(), NULL, NULL, &status), std::free};

    return (status == 0) ? std::string(res.get()) : std::string(name);
  }
#else
  namespace swarm {
  std::string internal_demangle(const char* name) { return std::string(name); }
  }  // namespace swarm
#endif  // __GNUG__

 public:
  /// @brief Retrieves the collision configuration for a specific type.
  ///
  /// @param type The type_index of the object to retrieve configuration for.
  /// @return The CollisionConfig structure containing the collision settings
  /// for the type.
  template <typename T>
  static CollisionConfig getCollisionConfig() {
    std::string type_name = typeid(T).name();
    auto it = configurations_.find(type_name);
    if (it != configurations_.end()) {
      return it->second;
    } else {
      std::cout << "Type not registered: " << internal_demangle(type_name)
                << std::endl;
      throw std::runtime_error("Type not registered");
    }
  }

  /// @brief Registers a new type and its collision partners in the system.
  ///
  /// This method registers a new type with its collision category bit and
  /// partners. It prints details about the registration process and ensures
  /// that the type is added with a unique category bit, which is then left
  /// shifted for the next registration.
  ///
  /// @param type The type_index of the new type to register.
  /// @param partners A vector of type_index specifying the collision partners
  /// for the new type.
  template <typename T>
  static void registerType(const std::vector<std::string>& partners) {
    std::string type_name = TypeKey<T>::value();
    spdlog::info("Registering type {}", internal_demangle(type_name));

    if (configurations_.find(type_name) == configurations_.end()) {
      uint16_t category_bit = next_category_bit_;
      std::bitset<16> catBits(category_bit);
      spdlog::info("Registering type {} with category bit {}",
                   internal_demangle(type_name), category_bit);

      next_category_bit_ <<= 1;
      configurations_[type_name] = {category_bit, 0};
    }
    collision_partners_[type_name] = partners;

    updateMaskBits();
    for (const auto& config : configurations_) {
      spdlog::info("Now registered types: {}", internal_demangle(config.first));

      std::bitset<16> catBits(config.second.categoryBits);
      spdlog::info("Mask bits: {}", config.second.maskBits);
    }
  }
};
}  // namespace swarm
#endif  // SWARM_UTILS_COLLISION_MANAGER_H