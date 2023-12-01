// PheremoneBehaviour.h
#pragma once
#include <box2d/b2_math.h>
#include <box2d/box2d.h>

#include <map>

#include "SwarmBehaviour.h"

class Drone;  // Forward declaration

struct PheremoneParameters {
  float decayRate;
};

class PheremoneBehaviour : public SwarmBehaviour {
 private:
  struct Pheremone {
    b2Vec2 position;
    float intensity;
  };

  std::map<int, Pheremone> pheremones;
  int pheremoneCount = 0;

  PheremoneParameters params;

 public:
  PheremoneBehaviour(const PheremoneParameters &params) : params(params) {}

  void execute(std::vector<Drone *> &drones, Drone *currentDrone) override;

 private:
  void updatePheremones();
  void layPheremone(const b2Vec2 &position);
};