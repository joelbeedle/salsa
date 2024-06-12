#ifndef SWARM_TARGET_H
#define SWARM_TARGET_H

#include <typeindex>

#include "core/entity.h"
#include "utils/collision_manager.h"

namespace swarm {
class Target : public Entity {
 private:
  float radius_;

 public:
  Target(b2World *world, const b2Vec2 &position, float radius);

  virtual void create_fixture() override {
    b2CircleShape shape;
    shape.m_radius = radius_;

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &shape;
    fixtureDef.isSensor = true;

    CollisionConfig config =
        CollisionManager::getCollisionConfig(typeid(Target));
    fixtureDef.filter.categoryBits = config.categoryBits;
    fixtureDef.filter.maskBits = config.maskBits;

    body_->CreateFixture(&fixtureDef);
  }
};

}  // namespace swarm

#endif  // SWARM_TARGET_H