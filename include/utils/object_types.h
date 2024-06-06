// ObjectTypes.h
#ifndef SWARM_SIM_UTILS_OBJECT_TYPES_H
#define SWARM_SIM_UTILS_OBJECT_TYPES_H

struct Drone;
struct Tree;

namespace swarm_sim {
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

}  // namespace swarm_sim

#endif  // SWARM_SIM_UTILS_OBJECT_TYPES_H