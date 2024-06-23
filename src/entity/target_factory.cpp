#include "entity/target_factory.h"

namespace swarm {
std::map<std::string, std::function<std::shared_ptr<Target>(std::any)>>
    TargetFactory::registry;
}  // namespace swarm