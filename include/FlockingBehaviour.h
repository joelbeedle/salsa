// FlockingBehaviour.h
#pragma once

#include <box2d/b2_math.h>
#include <box2d/box2d.h>

#include <vector>

#include "RayCastCallback.h"
#include "SwarmBehaviour.h"

class Drone;  // Forward declaration for Drone

class FlockingBehaviour : public SwarmBehaviour {
 private:
  float viewRange;
  float separationDistance;
  float alignmentWeight;
  float cohesionWeight;
  float separationWeight;
  float obstacleAvoidanceWeight;
  float maxSpeed;
  float maxForce;
  std::vector<b2Body *> obstacles;

 public:
  FlockingBehaviour(float viewRange, float separationDistance,
                    float alignmentWeight, float cohesionWeight,
                    float separationWeight, float obstacleAvoidanceWeight,
                    float maxSpeed, float maxForce)
      : viewRange(viewRange),
        separationDistance(separationDistance),
        alignmentWeight(alignmentWeight),
        cohesionWeight(cohesionWeight),
        separationWeight(separationWeight),
        obstacleAvoidanceWeight(obstacleAvoidanceWeight),
        maxForce(maxForce),
        maxSpeed(maxSpeed) {}

  void execute(std::vector<Drone *> &drones, Drone *currentDrone) override;

 private:
  b2Vec2 align(std::vector<b2Body *> &drones, Drone *currentDrone);
  b2Vec2 separate(std::vector<b2Body *> &drones, Drone *currentDrone);
  b2Vec2 cohere(std::vector<b2Body *> &drones, Drone *currentDrone);
  b2Vec2 avoidObstacles(std::vector<b2Vec2> &obstaclePoints,
                        Drone *currentDrone);
  void performRayCasting(Drone *currentDrone, RayCastCallback &callback);
  void clampMagnitude(b2Vec2 &vector, float maxMagnitude);
};