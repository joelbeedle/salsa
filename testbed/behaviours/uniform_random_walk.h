// UniformRandomWalkBehaviour.h
#ifndef SWARM_SIM_BEHAVIOURS_UNIFORM_RANDOM_WALK_H
#define SWARM_SIM_BEHAVIOURS_UNIFORM_RANDOM_WALK_H

#include <box2d/box2d.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "behaviours/behaviour.h"
#include "utils/raycastcallback.h"

class Drone;

namespace swarm {
// namespace behaviours {

class UniformRandomWalkBehaviour : public Behaviour {
 private:
  std::unordered_map<std::string, behaviour::Parameter *> parameters_;
  behaviour::Parameter max_magnitude_;
  behaviour::Parameter force_weight_;
  behaviour::Parameter obstacle_avoidance_weight_;
  behaviour::Parameter delta_time_{1.0f / 60.0f, 0.0f, 1.0f};
  struct DroneTimerInfo {
    float elapsedTimeSinceLastForce = 0.0f;
    float randomTimeInterval;
    b2Vec2 desiredVelocity;

    DroneTimerInfo() : randomTimeInterval(generateRandomTimeInterval()) {}
  };

  std::unordered_map<Drone *, DroneTimerInfo> droneTimers;

 public:
  UniformRandomWalkBehaviour(float maxMagnitude, float forceWeight,
                             float obstacleAvoidanceWeight)
      : max_magnitude_(maxMagnitude, 0.0f, 20.0f),
        force_weight_(forceWeight, 0.0f, 20.0f),
        obstacle_avoidance_weight_(obstacleAvoidanceWeight, 0.0f, 3.0f) {
    parameters_["Max Magnitude"] = &max_magnitude_;
    parameters_["Force Weight"] = &force_weight_;
    parameters_["Obstacle Avoidance Weight"] = &obstacle_avoidance_weight_;
    std::srand(std::time(nullptr));
  }

  std::unordered_map<std::string, behaviour::Parameter *> getParameters()
      override {
    return parameters_;
  }

  virtual ~UniformRandomWalkBehaviour() override {}

  void execute(const std::vector<std::unique_ptr<Drone>> &drones,
               Drone &currentDrone) override;

 private:
  static float generateRandomTimeInterval() {
    return static_cast<float>(std::rand()) / RAND_MAX * 5.0f;
  }
};

// }  // namespace behaviours
}  // namespace swarm

#endif  // SWARM_SIM_BEHAVIOURS_UNIFORM_RANDOM_WALK_H