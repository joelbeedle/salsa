/// @file collision_manager.h
/// @brief Handles collision configurations and updates for various types
/// deriving from `Entity`.
#ifndef SWARM_UTILS_COLLISION_MANAGER_H
#define SWARM_UTILS_COLLISION_MANAGER_H

#include <box2d/box2d.h>

#include <cstdint>
#include <map>
#include <typeindex>
#include <vector>

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
/// CollisionManager uses singleton pattern to maintain a consistent state
/// across the system. It offers functionality to register new types with
/// collision partners and update collision masks dynamically based on
/// registered types and their partners.
class CollisionManager {
 private:
  /// \brief Stores collision configuration for each registered type.
  static std::map<std::type_index, CollisionConfig> configurations_;
  /// \brief Maps each type to its collision partners.
  static std::map<std::type_index, std::vector<std::type_index>>
      collision_partners_;
  /// \brief Tracks the next available category bit for registering new types.
  static uint16_t next_category_bit_;

  /// @brief Updates the mask bits for all registered types based on their
  /// collision partners.
  ///
  /// This method iterates through each registered type, computes the mask bits
  /// by combining the category bits of its collision partners, and updates the
  /// mask bits accordingly.
  static void updateMaskBits();

 public:
  /// @brief Retrieves the collision configuration for a specific type.
  ///
  /// @param type The type_index of the object to retrieve configuration for.
  /// @return The CollisionConfig structure containing the collision settings
  /// for the type.
  static CollisionConfig getCollisionConfig(std::type_index type);

  /// \brief Registers a new type and its collision partners in the system.
  ///
  /// This method registers a new type with its collision category bit and
  /// partners. It prints details about the registration process and ensures
  /// that the type is added with a unique category bit, which is then left
  /// shifted for the next registration.
  ///
  /// \param type The type_index of the new type to register.
  /// \param partners A vector of type_index specifying the collision partners
  /// for the new type.
  static void registerType(std::type_index type,
                           const std::vector<std::type_index>& partners);
};
}  // namespace swarm
#endif  // SWARM_UTILS_COLLISION_MANAGER_H