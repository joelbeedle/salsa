#include "salsa/utils/collision_manager.h"

#include <iostream>

#include "spdlog/spdlog.h"

namespace swarm {

std::unordered_map<std::string, CollisionConfig, std::hash<std::string>,
                   std::equal_to<>>
    CollisionManager::configurations_;

std::unordered_map<std::string, std::vector<std::string>,
                   std::hash<std::string>, std::equal_to<>>
    CollisionManager::collision_partners_;

uint16_t CollisionManager::next_category_bit_ = 2;

void CollisionManager::updateMaskBits() {
  for (auto& config : configurations_) {
    uint16_t mask_bits = 1;
    for (auto& partner_type : collision_partners_[config.first]) {
      if (configurations_.find(partner_type) != configurations_.end()) {
        mask_bits |= configurations_[partner_type].categoryBits;
      }
    }
    configurations_[config.first].maskBits = mask_bits;
  }
}

}  // namespace swarm