// SwarmBehaviour.h
#pragma once
#include <box2d/box2d.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "utils/raycastcallback.h"
class Drone;

struct ParameterDefinition {
  float *value;
  float minSetting;
  float maxSetting;
};

class SwarmBehaviour {
 public:
  virtual ~SwarmBehaviour() = default;

  virtual void execute(const std::vector<std::unique_ptr<Drone>> &drones,
                       Drone &currentDrone) = 0;

  virtual std::unordered_map<std::string, ParameterDefinition>
  getParameters() = 0;

  virtual void clean(const std::vector<std::unique_ptr<Drone>> &drones) {}

 protected:
  void clampMagnitude(b2Vec2 &vector, const float maxMagnitude) {
    float lengthSquared = vector.LengthSquared();
    if (lengthSquared > maxMagnitude * maxMagnitude && lengthSquared > 0) {
      vector.Normalize();
      vector *= maxMagnitude;
    }
  }
  b2Vec2 avoidDrones(std::vector<b2Body *> &neighbours, Drone &currentDrone);
  b2Vec2 avoidObstacles(std::vector<b2Vec2> &obstaclePoints,
                        Drone &currentDrone);
  b2Vec2 steerTo(b2Vec2 target, Drone &currentDrone);
  void performRayCasting(Drone &currentDrone, RayCastCallback &callback);
};