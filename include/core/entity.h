#ifndef SWARM_ENTITY_H
#define SWARM_ENTITY_H

#include <box2d/box2d.h>

#include <vector>

#include "core/data.h"
namespace swarm {
class Entity {
 protected:
  b2Body *body_;
  b2World *world_;
  std::vector<std::shared_ptr<Observer>> observers;

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

  Entity() = default;

  virtual ~Entity() {
    if (body_) {
      world_->DestroyBody(body_);
    }
  }

  virtual void create_fixture() = 0;

  void addObserver(std::shared_ptr<Observer> observer) {
    observers.push_back(observer);
  }

  void notifyAll(const nlohmann::json &message) {
    for (auto &observer : observers) {
      observer->update(message);
    }
  }
};

}  // namespace swarm

#endif  // SWARM_ENTITY_H
