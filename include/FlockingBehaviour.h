// FlockingBehaviour.h
#pragma once

#include <box2d/b2_math.h>
#include <box2d/box2d.h>

#include <unordered_map>
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
  FlockingParameters params;

  // Define the parameter names and their expected min and max values
  // (for the UI)
  std::unordered_map<std::string, ParameterDefinition> cleanParams = {
      {"Separation Distance", {&params.separationDistance, 0.0f, 100.0f}},
      {"Alignment Weight", {&params.alignmentWeight, 0.0f, 2.0f}},
      {"Cohesion Weight", {&params.cohesionWeight, 0.0f, 2.0f}},
      {"Separation Weight", {&params.separationWeight, 0.0f, 2.0f}},
      {"Obstacle Avoidance Weight",
       {&params.obstacleAvoidanceWeight, 0.0f, 5.0f}}};
  std::vector<b2Body *> obstacles;

 public:
  FlockingBehaviour(const FlockingParameters &params) : params(params) {}

  void execute(std::vector<Drone *> &drones, Drone *currentDrone) override;

  std::unordered_map<std::string, ParameterDefinition> getParameters()
      override {
    return cleanParams;
  }

 private:
  b2Vec2 align(std::vector<b2Body *> &drones, Drone *currentDrone);
  b2Vec2 separate(std::vector<b2Body *> &drones, Drone *currentDrone);
  b2Vec2 cohere(std::vector<b2Body *> &drones, Drone *currentDrone);
  b2Vec2 avoidObstacles(std::vector<b2Vec2> &obstaclePoints,
                        Drone *currentDrone);
  void performRayCasting(Drone *currentDrone, RayCastCallback &callback);
};