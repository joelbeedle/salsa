// PheromoneBehaviour.h
#ifndef SWARM_SIM_BEHAVIOURS_Pheromone_AVOIDANCE_H
#define SWARM_SIM_BEHAVIOURS_Pheromone_AVOIDANCE_H

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
class PheromoneBehaviour : public Behaviour {
 private:
  struct Pheromone {
    b2Vec2 position;
    float intensity;
  };

  std::map<int, Pheromone> pheromones;
  int pheromoneCount = 0;
  std::unordered_map<std::string, behaviour::Parameter *> parameters_;
  behaviour::Parameter decay_rate_;
  behaviour::Parameter obstacle_avoidance_weight_;

 public:
  PheromoneBehaviour(float decayRate, float obstacleAvoidanceWeight)
      : decay_rate_(decayRate, 0.0f, 50.0f),
        obstacle_avoidance_weight_(obstacleAvoidanceWeight, 0.0f, 3.0f) {
    // Register parameters in the map
    parameters_["Decay Rate"] = &decay_rate_;
    parameters_["Obstacle Avoidance Weight"] = &obstacle_avoidance_weight_;
  }

  void execute(const std::vector<std::unique_ptr<Drone>> &drones,
               Drone &currentDrone) override;

  std::unordered_map<std::string, behaviour::Parameter *> getParameters()
      override {
    return parameters_;
  }

 private:
  void updatePheromones();
  void layPheromone(const b2Vec2 &position);
};

// }  // namespace behaviours
}  // namespace swarm

#endif  // SWARM_SIM_BEHAVIOURS_Pheromone_AVOIDANCE_H