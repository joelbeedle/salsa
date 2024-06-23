#include "entity/target.h"

#include "entity/drone.h"

namespace swarm {
Target::Target(b2World *world, const b2Vec2 &position, float radius)
    : Entity(world, position, true) {
  create_fixture();
}
}  // namespace swarm