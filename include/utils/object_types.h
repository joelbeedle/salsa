// ObjectTypes.h
#ifndef SWARM_SIM_UTILS_OBJECT_TYPES_H
#define SWARM_SIM_UTILS_OBJECT_TYPES_H

struct Drone;
struct Tree;

namespace swarm {
enum class ObjectType {
  Drone,
  Tree,
};

struct UserData {
  ObjectType type;
  union {
    Drone* drone;
    Tree* tree;
  };
};

}  // namespace swarm

#endif  // SWARM_SIM_UTILS_OBJECT_TYPES_H