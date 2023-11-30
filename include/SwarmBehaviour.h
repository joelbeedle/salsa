// SwarmBehaviour.h
#pragma once
#include <vector>

class Drone;

class SwarmBehaviour {
 public:
  virtual ~SwarmBehaviour() = default;

  virtual void execute(std::vector<Drone *> &drones, Drone *currentDrone) = 0;

  void clampMagnitude(b2Vec2 &vector, float maxMagnitude) {
    float lengthSquared = vector.LengthSquared();
    if (lengthSquared > maxMagnitude * maxMagnitude && lengthSquared > 0) {
      vector.Normalize();
      vector *= maxMagnitude;
    }
  }
};