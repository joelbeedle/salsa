// Tree.h
#pragma once
#include <box2d/box2d.h>

class Tree {
 private:
  b2Body *body;
  bool diseased;
  bool mapped;
  float radius;
  int treeID;

 public:
  Tree(b2World *world, int treeID, const b2Vec2 &position, bool diseased,
       bool mapped = false, float radius = 1.0f);
  ~Tree();

  // Accessors and Mutators
  bool isDiseased() { return diseased; }
  void setDiseased(bool isDiseased) { diseased = isDiseased; }
  bool isMapped() { return mapped; }
  void setMapped(bool isMapped) { mapped = isMapped; }
  b2Body *getBody() { return body; }
  void setID(int newID) { treeID = newID; }
  int getID() { return treeID; }
  void render();
};