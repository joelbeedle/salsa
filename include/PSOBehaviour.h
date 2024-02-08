// PSOBehaviour.h
#pragma once
#include <box2d/box2d.h>

#include <unordered_map>
#include <vector>

#include "RayCastCallback.h"
#include "SwarmBehaviour.h"

class Drone;

struct PSOParameters {
  float cognitiveComponent;
  float socialComponent;
  float inertiaWeight;
};

class PSOBehaviour : public SwarmBehaviour {
 private:
  PSOParameters params;
  int globalBestScore;
  b2Vec2 globalBestPosition;
  std::unordered_map<Drone *, b2Vec2>
      personalBestPositions;  // Best positions based on exploration success
  std::unordered_map<Drone *, int>
      personalBestScores;  // Tracks the best score (e.g., number of trees
                           // scanned) for each drone

  // Define the parameter names and their expected min and max values
  // (for the UI)
  std::unordered_map<std::string, ParameterDefinition> cleanParams = {
      {"Cognitive Component", {&params.cognitiveComponent, 0.0f, 1.0f}},
      {"Social Component", {&params.socialComponent, 0.0f, 1.0f}},
      {"Inertia Weight", {&params.inertiaWeight, 0.0f, 1.0f}}};
  std::vector<b2Body *> obstacles;

 public:
  PSOBehaviour(const PSOParameters &params) : params(params) {}

  void execute(std::vector<Drone *> &drones, Drone *currentDrone) override;

  std::unordered_map<std::string, ParameterDefinition> getParameters()
      override {
    return cleanParams;
  }

 private:
  void updatePersonalBest(Drone *drone);
  float randomFactor();
  int calculateScoreForDrone(Drone *drone);
  b2Vec2 avoidObstacles(std::vector<b2Vec2> &obstaclePoints,
                        Drone *currentDrone);
  void performRayCasting(Drone *currentDrone, RayCastCallback &callback);
};