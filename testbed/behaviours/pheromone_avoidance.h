// PheromoneBehaviour.h
#ifndef swarm_BEHAVIOURS_Pheromone_AVOIDANCE_H
#define swarm_BEHAVIOURS_Pheromone_AVOIDANCE_H

#include <box2d/b2_math.h>
#include <box2d/box2d.h>

#include <map>
#include <memory>
#include <unordered_map>

#include "behaviours/behaviour.h"
#include "utils/raycastcallback.h"

class Drone;  // Forward declaration

namespace swarm {
// namespace behaviours {
struct PheromoneParameters {
  float decayRate;
  float obstacleAvoidanceWeight;
};

class PheromoneBehaviour : public Behaviour {
 private:
  struct Pheromone {
    b2Vec2 position;
    float intensity;
  };

  std::map<int, Pheromone> pheromones;
  int pheromoneCount = 0;

  PheromoneParameters params;
  // Define the parameter names and their expected min and max values
  // (for the UI)
  std::unordered_map<std::string, ParameterDefinition> cleanParams = {
      {"Decay Rate", {&params.decayRate, 0.0f, 1.0f}},
      {"Obstacle Avoidance Weight",
       {&params.obstacleAvoidanceWeight, 0.0f, 3.0f}}};

 public:
  PheromoneBehaviour(const PheromoneParameters &params) : params(params) {}

  void execute(const std::vector<std::unique_ptr<Drone>> &drones,
               Drone &currentDrone) override;

  std::unordered_map<std::string, ParameterDefinition> getParameters()
      override {
    return cleanParams;
  }

 private:
  void updatePheromones();
  void layPheromone(const b2Vec2 &position);
};

// }  // namespace behaviours
}  // namespace swarm

#endif  // swarm_BEHAVIOURS_Pheromone_AVOIDANCE_H