// FlockingBehaviour.h
#pragma once

#include <box2d/b2_math.h>
#include <box2d/box2d.h>

#include <vector>

#include "RayCastCallback.h"
#include "SwarmBehaviour.h"

class Drone;  // Forward declaration for Drone

struct FlockingParameters {
  float separationDistance;
  float alignmentWeight;
  float cohesionWeight;
  float separationWeight;
  float obstacleAvoidanceWeight;
};

class FlockingBehaviour : public SwarmBehaviour {
 private:
  float separationDistance;
  float alignmentWeight;
  float cohesionWeight;
  float separationWeight;
  float obstacleAvoidanceWeight;
  std::vector<b2Body *> obstacles;

 public:
  FlockingBehaviour(const FlockingParameters &params)
      : separationDistance(params.separationDistance),
        alignmentWeight(params.alignmentWeight),
        cohesionWeight(params.cohesionWeight),
        separationWeight(params.separationWeight),
        obstacleAvoidanceWeight(params.obstacleAvoidanceWeight) {}

  void execute(std::vector<Drone *> &drones, Drone *currentDrone) override;

 private:
  b2Vec2 align(std::vector<b2Body *> &drones, Drone *currentDrone);
  b2Vec2 separate(std::vector<b2Body *> &drones, Drone *currentDrone);
  b2Vec2 cohere(std::vector<b2Body *> &drones, Drone *currentDrone);
  b2Vec2 avoidObstacles(std::vector<b2Vec2> &obstaclePoints,
                        Drone *currentDrone);
  void performRayCasting(Drone *currentDrone, RayCastCallback &callback);
};