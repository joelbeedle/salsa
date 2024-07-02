#include "salsa/entity/target.h"

#include "salsa/entity/drone.h"

namespace swarm {
Target::Target(b2World *world, const b2Vec2 &position, float radius)
    : Entity(world, position, true, radius, swarm::get_type<Target>()) {}
}  // namespace swarm