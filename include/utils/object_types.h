// ObjectTypes.h
#ifndef SWARM_SIM_UTILS_OBJECT_TYPES_H
#define SWARM_SIM_UTILS_OBJECT_TYPES_H

#include "entity/drone.h"
#include "entity/entity.h"
#include "entity/target.h"
namespace swarm {

enum class ObjectType {
  Drone,
  Target,
};

std::string demangle(const char *name);

template <class T>
std::string type(const T &t) {
  return demangle(typeid(t).name());
}

template <typename T>
std::string get_type() {
  return demangle(typeid(T).name());
}

struct UserData {
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