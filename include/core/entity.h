#ifndef SWARM_ENTITY_H
#define SWARM_ENTITY_H

#include <box2d/box2d.h>
namespace swarm {
class Entity {
 protected:
  b2Body *body_;
  b2World *world_;

 public:
  Entity(b2World *world, const b2Vec2 &position, bool is_static)
      : world_(world) {
    b2BodyDef bodyDef;
    if (is_static) {
      bodyDef.type = b2_staticBody;
    } else {
      bodyDef.type = b2_dynamicBody;
    }
    bodyDef.position = position;
    this->body_ = world_->CreateBody(&bodyDef);
  }

  virtual ~Entity() {
    if (body_) {
      world_->DestroyBody(body_);
    }
  }

  virtual void create_fixture() = 0;
};

}  // namespace swarm

#endif  // SWARM_ENTITY_H
