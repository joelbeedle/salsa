// FlockingBehaviour.h
#ifndef swarm_BEHAVIOURS_LEVY_FLOCKING_H
#define swarm_BEHAVIOURS_LEVY_FLOCKING_H

#include <box2d/b2_math.h>
#include <box2d/box2d.h>

#include <iostream>
#include <random>
#include <unordered_map>
#include <vector>

#include "behaviours/behaviour.h"
#include "utils/raycastcallback.h"

class Drone;

namespace swarm {
// namespace behaviours {
struct LevyFlockingParameters {
  float separationDistance;
  float alignmentWeight;
  float cohesionWeight;
  float separationWeight;
  float levyWeight;
  float obstacleAvoidanceWeight;
};
class LevyFlockingBehaviour : public Behaviour {
 private:
  std::unordered_map<std::string, behaviour::Parameter *> parameters_;

  behaviour::Parameter separation_distance_{0.0f, 0.0f, 1000.0f};
  behaviour::Parameter alignment_weight_;
  behaviour::Parameter cohesion_weight_;
  behaviour::Parameter separation_weight_;
  behaviour::Parameter levy_weight_;
  behaviour::Parameter obstacle_avoidance_weight_;

  // Define the parameter names and their expected min and max values
  // (for the UI)
  std::vector<b2Body *> obstacles;

  struct DroneInfo {
    b2Vec2 levyPoint;
    b2Vec2 levyDirection;
    float accumulatedDistance = 0.0f;
    bool isExecuting = false;

    DroneInfo() : levyPoint(levy(2.1f)) {}

    float stepLength = 0.0f;
  };

  std::unordered_map<Drone *, DroneInfo> droneInformation;

 public:
  LevyFlockingBehaviour(float separationDistance, float alignmentWeight,
                        float cohesionWeight, float separationWeight,
                        float levyWeight, float obstacleAvoidanceWeight)
      : separation_distance_(separationDistance, 0.0f, 1000.0f),
        alignment_weight_(alignmentWeight, 0.0f, 2.0f),
        cohesion_weight_(cohesionWeight, 0.0f, 2.0f),
        separation_weight_(separationWeight, 0.0f, 5.0f),
        levy_weight_(levyWeight, 0.0f, 5.0f),
        obstacle_avoidance_weight_(obstacleAvoidanceWeight, 0.0f, 5.0f) {
    // Register parameters in the map
    parameters_["Separation Distance"] = &separation_distance_;
    parameters_["Alignment Weight"] = &alignment_weight_;
    parameters_["Cohesion Weight"] = &cohesion_weight_;
    parameters_["Separation Weight"] = &separation_weight_;
    parameters_["Levy Weight"] = &levy_weight_;
    parameters_["Obstacle Avoidance Weight"] = &obstacle_avoidance_weight_;
  }

  void execute(const std::vector<std::unique_ptr<Drone>> &drones,
               Drone &currentDrone) override;

  std::unordered_map<std::string, behaviour::Parameter *> getParameters()
      override {
    return parameters_;
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

// }  // namespace behaviours
}  // namespace swarm

#endif  // swarm_BEHAVIOURS_LEVY_FLOCKING_H