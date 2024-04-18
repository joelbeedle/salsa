// FlockingBehaviour.h
#pragma once

#include <box2d/b2_math.h>
#include <box2d/box2d.h>

#include <iostream>
#include <random>
#include <unordered_map>
#include <vector>

#include "RayCastCallback.h"
#include "SwarmBehaviour.h"

class Drone;
struct LevyFlockingParameters {
  float separationDistance;
  float alignmentWeight;
  float cohesionWeight;
  float separationWeight;
  float levyWeight;
  float obstacleAvoidanceWeight;
};
class LevyFlockingBehaviour : public SwarmBehaviour {
 private:
  LevyFlockingParameters params;

  // Define the parameter names and their expected min and max values
  // (for the UI)
  std::unordered_map<std::string, ParameterDefinition> cleanParams = {
      {"Separation Distance", {&params.separationDistance, 0.0f, 100.0f}},
      {"Alignment Weight", {&params.alignmentWeight, 0.0f, 2.0f}},
      {"Cohesion Weight", {&params.cohesionWeight, 0.0f, 2.0f}},
      {"Separation Weight", {&params.separationWeight, 0.0f, 2.0f}},
      {"Levy Weight", {&params.levyWeight, 0.0f, 10.0f}},

      {"Obstacle Avoidance Weight",
       {&params.obstacleAvoidanceWeight, 0.0f, 5.0f}}};
  std::vector<b2Body *> obstacles;

  struct DroneInfo {
    b2Vec2 levyPoint;
    b2Vec2 levyDirection;
    float accumulatedDistance = 0.0f;
    bool isExecuting = false;

    DroneInfo() : levyPoint(levy(2.8f)) {}

    float stepLength = sqrt(pow(levyPoint.x, 2) + pow(levyPoint.y, 2));
  };

  std::unordered_map<Drone *, DroneInfo> droneInformation;

 public:
  LevyFlockingBehaviour(const LevyFlockingParameters &params)
      : params(params) {}

  void execute(const std::vector<std::unique_ptr<Drone>> &drones,
               Drone &currentDrone) override;

  std::unordered_map<std::string, ParameterDefinition> getParameters()
      override {
    return cleanParams;
  }

 private:
  b2Vec2 align(std::vector<b2Body *> &drones, Drone &currentDrone);
  b2Vec2 separate(std::vector<b2Body *> &drones, Drone &currentDrone);
  b2Vec2 cohere(std::vector<b2Body *> &drones, Drone &currentDrone);

  static b2Vec2 levy(float mu) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.0, 1.0);

    double u1 = dis(gen);
    double u2 = dis(gen);
    double u3 = dis(gen);

    double U1 = u1 * (M_PI / 2);
    double U2 = (u2 + 1) / 2;
    double phi = u3 * M_PI;

    double a = sin((mu - 1) * U1);
    double b = pow(cos(U1), 1 / (1 - mu));
    double c = cos((2 - mu) * U1);
    double d = (a / b) * (c / U2);
    double r = pow(d, (2 - mu) / (mu - 1));

    double x = r * cos(phi);
    double y = r * sin(phi);
    return b2Vec2(x, y);
  }

  b2Vec2 generateRandomDirection() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0, 2 * M_PI);

    double theta = dis(gen);
    return b2Vec2(cos(theta), sin(theta));
  }
};