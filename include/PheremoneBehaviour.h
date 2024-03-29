// PheremoneBehaviour.h
#pragma once
#include <box2d/b2_math.h>
#include <box2d/box2d.h>

#include <map>
#include <unordered_map>

#include "RayCastCallback.h"
#include "SwarmBehaviour.h"

class Drone;  // Forward declaration

struct PheremoneParameters {
  float decayRate;
  float obstacleAvoidanceWeight;
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
  // Define the parameter names and their expected min and max values
  // (for the UI)
  std::unordered_map<std::string, ParameterDefinition> cleanParams = {
      {"Decay Rate", {&params.decayRate, 0.0f, 1.0f}},
      {"Obstacle Avoidance Weight",
       {&params.obstacleAvoidanceWeight, 0.0f, 3.0f}}};

 public:
  PheremoneBehaviour(const PheremoneParameters &params) : params(params) {}

  void execute(const std::vector<std::unique_ptr<Drone>> &drones,
               Drone &currentDrone) override;

  std::unordered_map<std::string, ParameterDefinition> getParameters()
      override {
    return cleanParams;
  }

 private:
  void updatePheremones();
  void layPheremone(const b2Vec2 &position);
};