#include "utils/collision_manager.h"

#include <iostream>
namespace swarm {

std::map<std::type_index, CollisionConfig> CollisionManager::configurations_;
std::map<std::type_index, std::vector<std::type_index>>
    CollisionManager::collision_partners_;
uint16_t CollisionManager::next_category_bit_ = 0x0001;

void CollisionManager::updateMaskBits() {
  for (auto &config : configurations_) {
    uint16_t mask_bits = 0;
    for (auto &partner_type : collision_partners_[config.first]) {
      if (configurations_.find(partner_type) != configurations_.end()) {
        mask_bits |= configurations_[partner_type].categoryBits;
      }
    }
    configurations_[config.first].maskBits = mask_bits;
  }
}

CollisionConfig CollisionManager::getCollisionConfig(std::type_index type) {
  return configurations_[type];
}

void CollisionManager::registerType(
    std::type_index type, const std::vector<std::type_index> &partners) {
  for (const auto &config : configurations_) {
    std::cout << "Already registered type: " << config.first.name()
              << std::endl;
  }
  std::cout << "Registering type " << type.name() << std::endl;

  if (configurations_.find(type) == configurations_.end()) {
    uint16_t category_bit = next_category_bit_;
    std::bitset<16> catBits(category_bit);
    std::cout << "Registering type " << type.name() << " with category bit "
              << category_bit << std::endl;
    std::cout << catBits << std::endl;

    next_category_bit_ <<= 1;
    configurations_[type] = {category_bit, 0};
  }
  collision_partners_[type] = partners;
  for (const auto &config : configurations_) {
    std::cout << "Now registered types: " << config.first.name() << std::endl;
  }

  updateMaskBits();
}
}  // namespace swarm