#ifndef swarm_BEHAVIOURS_FLOCKING_H
#define swarm_BEHAVIOURS_FLOCKING_H

#include <box2d/b2_math.h>
#include <box2d/box2d.h>

#include <unordered_map>
#include <vector>

#include "behaviours/behaviour.h"
#include "behaviours/parameter.h"
#include "utils/raycastcallback.h"

class Drone;  // Forward declaration for Drone

namespace swarm {
// namespace behaviours {

class FlockingBehaviour : public Behaviour {
 private:
  std::unordered_map<std::string, behaviour::Parameter *> parameters_;

  std::vector<b2Body *> obstacles;

  behaviour::Parameter separation_distance_;
  behaviour::Parameter alignment_weight_;
  behaviour::Parameter cohesion_weight_;
  behaviour::Parameter separation_weight_;
  behaviour::Parameter obstacle_avoidance_weight_;

 public:
  FlockingBehaviour(float separationDistance, float alignmentWeight,
                    float cohesionWeight, float separationWeight,
                    float obstacleAvoidanceWeight)
      : separation_distance_(separationDistance, 0.0f, 1000.0f),
        alignment_weight_(alignmentWeight, 0.0f, 2.0f),
        cohesion_weight_(cohesionWeight, 0.0f, 2.0f),
        separation_weight_(separationWeight, 0.0f, 5.0f),
        obstacle_avoidance_weight_(obstacleAvoidanceWeight, 0.0f, 5.0f) {
    // Register parameters in the map
    parameters_["Separation Distance"] = &separation_distance_;
    parameters_["Alignment Weight"] = &alignment_weight_;
    parameters_["Cohesion Weight"] = &cohesion_weight_;
    parameters_["Separation Weight"] = &separation_weight_;
    parameters_["Obstacle Avoidance Weight"] = &obstacle_avoidance_weight_;
  }

  void execute(const std::vector<std::unique_ptr<Drone>> &drones,
               Drone &currentDrone) override;

  std::unordered_map<std::string, behaviour::Parameter *> getParameters()
      override {
    return parameters_;
  }

 private:
  b2Vec2 align(const std::vector<std::unique_ptr<Drone>> &drones,
               Drone &currentDrone);
  b2Vec2 separate(const std::vector<std::unique_ptr<Drone>> &drones,
                  Drone &currentDrone);
  b2Vec2 cohere(const std::vector<std::unique_ptr<Drone>> &drones,
                Drone &currentDrone);

  b2Vec2 avoidWall(b2Vec2 point, Drone &currentDrone);
};

// }  // namespace behaviours
}  // namespace swarm

#endif  // swarm_BEHAVIOURS_FLOCKING_H