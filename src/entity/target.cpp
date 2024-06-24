#include "entity/target.h"

#include "entity/drone.h"

namespace swarm {
Target::Target(b2World *world, const b2Vec2 &position, float radius,
               std::string type_name)
    : Entity(world, position, true, radius, type_name), radius_(radius) {}
}  // namespace swarm