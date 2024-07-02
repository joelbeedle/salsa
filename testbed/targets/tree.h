// Tree.h
#ifndef TREE_H
#define TREE_H
#include <box2d/box2d.h>
#include <salsa/salsa.h>

class Tree : public salsa::Target {
 private:
  bool diseased;
  bool mapped;
  bool currentlyMapped = false;
  float radius;
  int numMapped = 0;

 public:
  Tree(b2World *world = nullptr, const b2Vec2 &position = b2Vec2(0, 0),
       int treeID = 0, bool diseased = false, bool mapped = false,
       float radius = 1.0f);
  ~Tree();

  void create_fixture();

  std::string getType() const override { return "Tree"; }

  // Accessors and Mutators
  bool isDiseased() { return diseased; }
  void setDiseased(bool isDiseased) { diseased = isDiseased; }
  bool isMapped() { return mapped; }
  void setMapped(bool isMapped) { setFound(isMapped); }
  b2Body *getBody() { return body_; }
  void addNumMapped() {
    if (!currentlyMapped) {
      numMapped++;
      currentlyMapped = true;
    }
  }
  void resetMapping() { currentlyMapped = false; }
  void resetNumMapped() { numMapped = 0; }
  int getNumMapped() { return numMapped; }
};

#endif  // TREE_H
