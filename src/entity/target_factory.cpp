#include "salsa/entity/target_factory.h"

namespace salsa {
std::map<std::string, TargetFactory::TargetCreateFunc> TargetFactory::registry;

std::vector<std::string> TargetFactory::getTargetNames() {
  std::vector<std::string> names;
  names.reserve(registry.size());
for (const auto& [fst, snd] : registry) {
    names.push_back(fst);
  }
  return names;
}

}  // namespace salsa