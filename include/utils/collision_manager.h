#ifndef SWARM_UTILS_COLLISION_MANAGER_H
#define SWARM_UTILS_COLLISION_MANAGER_H

#include <box2d/box2d.h>

#include <cstdint>
#include <map>
#include <typeindex>
#include <vector>

namespace swarm {

struct CollisionConfig {
  uint16_t categoryBits;
  uint16_t maskBits;
};

class CollisionManager {
 private:
  static std::map<std::type_index, CollisionConfig> configurations_;
  static std::map<std::type_index, std::vector<std::type_index>>
      collision_partners_;
  static uint16_t next_category_bit_;

  static void updateMaskBits();

 public:
  static CollisionConfig getCollisionConfig(std::type_index type);

  static void registerType(std::type_index type,
                           const std::vector<std::type_index>& partners);
};
}  // namespace swarm
#endif  // SWARM_UTILS_COLLISION_MANAGER_H