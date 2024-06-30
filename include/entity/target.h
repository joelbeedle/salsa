#ifndef SWARM_TARGET_H
#define SWARM_TARGET_H

#include <typeindex>

#include "entity/entity.h"
#include "utils/collision_manager.h"

namespace swarm {
class Target : public Entity {
 protected:
  bool found_ = false;

 public:
  Target(b2World *world, const b2Vec2 &position, float radius);

  virtual std::string getType() const = 0;

  b2Vec2 getPosition() const { return body_->GetPosition(); }
  float getRadius() const { return radius_; }

  bool isFound() const { return found_; }
  void setFound(bool found) { found_ = found; }
};

}  // namespace swarm

#endif  // SWARM_TARGET_H