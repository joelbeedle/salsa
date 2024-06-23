#include "entity/target_factory.h"

namespace swarm {
std::map<std::string, TargetFactory::TargetCreateFunc> TargetFactory::registry;

}  // namespace swarm