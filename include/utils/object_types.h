// ObjectTypes.h
#ifndef SWARM_SIM_UTILS_OBJECT_TYPES_H
#define SWARM_SIM_UTILS_OBJECT_TYPES_H

#include "entity/drone.h"
#include "entity/entity.h"

namespace swarm {
enum class ObjectType {
  Drone,
  Target,
};

struct UserData {
  ObjectType type;
  Entity *object;  // Point to any object derived from EnvironmentObject

  UserData() : object(nullptr) {}
  explicit UserData(Entity *obj) : object(obj) {}

  template <typename T>
  T *as() {
    return static_cast<T *>(object);  // Safely cast to the requested type
  }
};

}  // namespace swarm

#endif  // SWARM_SIM_UTILS_OBJECT_TYPES_H