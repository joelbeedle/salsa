#include "salsa/entity/target_factory.h"

namespace swarm {
std::map<std::string, TargetFactory::TargetCreateFunc> TargetFactory::registry;

std::vector<std::string> TargetFactory::getTargetNames() {
  std::vector<std::string> names;
  for (const auto& entry : registry) {
    names.push_back(entry.first);
  }
  return names;
}

}  // namespace swarm