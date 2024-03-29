// UniformRandomWalkBehaviour.h
#pragma once

#include <box2d/box2d.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <string>
#include <unordered_map>
#include <vector>

#include "RayCastCallback.h"
#include "SwarmBehaviour.h"

class Drone;

struct UniformRandomWalkParameters {
  float maxMagnitude;
  float forceWeight;
  float obstacleAvoidanceWeight;
  float deltaTime = 1.0f / 60.0f;
};

class UniformRandomWalkBehaviour : public SwarmBehaviour {
 private:
  UniformRandomWalkParameters params;
  std::unordered_map<std::string, ParameterDefinition> cleanParams = {
      {"Max Magnitude", {&params.maxMagnitude, 0.0f, 20.0f}},
      {"Force Weight", {&params.forceWeight, 0.0f, 20.0f}},
      {"Obstacle Avoidance Weight",
       {&params.obstacleAvoidanceWeight, 0.0f, 3.0f}}};

  struct DroneTimerInfo {
    float elapsedTimeSinceLastForce = 0.0f;
    float randomTimeInterval;
    b2Vec2 desiredVelocity;

    DroneTimerInfo() : randomTimeInterval(generateRandomTimeInterval()) {}
  };

  std::unordered_map<Drone *, DroneTimerInfo> droneTimers;

 public:
  UniformRandomWalkBehaviour(const UniformRandomWalkParameters &params);

  std::unordered_map<std::string, ParameterDefinition> getParameters()
      override {
    return cleanParams;
  }

  virtual ~UniformRandomWalkBehaviour() override {}

  void execute(std::vector<std::unique_ptr<Drone>> &drones,
               Drone *currentDrone) override;

 private:
  // Function to generate a random time interval between 0 and 10 seconds
  static float generateRandomTimeInterval() {
    return static_cast<float>(std::rand()) / RAND_MAX * 5.0f;
  }
};
