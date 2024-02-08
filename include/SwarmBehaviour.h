// SwarmBehaviour.h
#pragma once
#include <box2d/box2d.h>

#include <string>
#include <unordered_map>
#include <vector>
class Drone;

struct ParameterDefinition {
  float *value;
  float minSetting;
  float maxSetting;
};

class SwarmBehaviour {
 public:
  virtual ~SwarmBehaviour() = default;

  virtual void execute(std::vector<Drone *> &drones, Drone *currentDrone) = 0;

  virtual std::unordered_map<std::string, ParameterDefinition>
  getParameters() = 0;

 protected:
  void clampMagnitude(b2Vec2 &vector, float maxMagnitude) {
    float lengthSquared = vector.LengthSquared();
    if (lengthSquared > maxMagnitude * maxMagnitude && lengthSquared > 0) {
      vector.Normalize();
      vector *= maxMagnitude;
    }
  }
  b2Vec2 avoidDrones(std::vector<b2Body *> &neighbours, Drone *currentDrone);
  b2Vec2 avoidObstacles(std::vector<b2Vec2> &obstaclePoints,
                        Drone *currentDrone);
};