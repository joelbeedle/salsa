#include "entity/target.h"

#include "entity/drone.h"

namespace swarm {
Target::Target(b2World *world, const b2Vec2 &position, float radius)
    : Entity(world, position, true, radius, swarm::get_type<Target>()) {
  b2CircleShape circleShape;
  circleShape.m_radius = radius_;
  b2FixtureDef fixtureDef;
  fixtureDef.shape = &circleShape;
  float area_m2 = M_PI * pow(radius_, 2);

  UserData *userData = new UserData();
  userData->object = this;

  fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(userData);
  body_->CreateFixture(&fixtureDef);
}
}  // namespace swarm