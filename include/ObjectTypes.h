// ObjectTypes.h
#pragma once

struct Drone;
struct Tree;

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
