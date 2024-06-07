// Tree.h
#ifndef TREE_H
#define TREE_H
#include <box2d/box2d.h>

class Tree {
 private:
  b2Body *body;
  bool diseased;
  bool mapped;
  bool currentlyMapped = false;
  float radius;
  int treeID;
  int numMapped = 0;

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
  void addNumMapped() {
    if (!currentlyMapped) {
      numMapped++;
      currentlyMapped = true;
    }
  }
  void resetMapping() { currentlyMapped = false; }
  void resetNumMapped() { numMapped = 0; }
  int getNumMapped() { return numMapped; }
  void render();
};
#endif // TREE_H
