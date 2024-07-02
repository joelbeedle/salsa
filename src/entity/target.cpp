#include "salsa/entity/target.h"

#include "salsa/entity/drone.h"

namespace salsa {
Target::Target(b2World *world, const b2Vec2 &position, float radius)
    : Entity(world, position, true, radius, salsa::get_type<Target>()) {}
}  // namespace salsa