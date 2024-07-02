/// @file target.h
/// @brief Defines the Target class for representing target objects in the
/// simulation.
#ifndef SWARM_TARGET_H
#define SWARM_TARGET_H

#include <typeindex>

#include "entity.h"
#include "salsa/utils/collision_manager.h"

namespace swarm {

/// @class Target
/// @brief Represents a target object in the simulation environment.
///
/// Target is a specialized entity that can be found or not found. Each target
/// has a type that is specific to its derived class.
class Target : public Entity {
 protected:
  bool found_ = false;

 public:
  /// @brief Constructor for the Target.
  /// @param world Pointer to the b2World where the target exists.
  /// @param position Initial position of the target.
  /// @param radius Radius of the target.
  Target(b2World *world, const b2Vec2 &position, float radius);

  virtual std::string getType() const = 0;

  bool isFound() const { return found_; }
  void setFound(bool found) { found_ = found; }
};

}  // namespace swarm

#endif  // SWARM_TARGET_H