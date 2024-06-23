#include <box2d/box2d.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/simulation.h"
namespace swarm {
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
               Drone &currentDrone) override {
    if (droneTimers.find(&currentDrone) == droneTimers.end()) {
      droneTimers[&currentDrone] = DroneTimerInfo();
      droneTimers[&currentDrone].desiredVelocity = currentDrone.getVelocity();
    }
    RayCastCallback callback;
    performRayCasting(currentDrone, callback);

    std::vector<b2Vec2> obstaclePoints = callback.obstaclePoints;
    std::vector<b2Body *> neighbours = callback.detectedDrones;

    DroneTimerInfo &timerInfo = droneTimers[&currentDrone];
    timerInfo.elapsedTimeSinceLastForce += delta_time_;
    b2Vec2 force = b2Vec2(0, 0);
    b2Vec2 steer = b2Vec2(0, 0);
    b2Vec2 obstacleAvoidance = avoidObstacles(obstaclePoints, currentDrone);
    b2Vec2 neighbourAvoidance = avoidDrones(neighbours, currentDrone);

    // Check if it's time to apply a new random force
    if (timerInfo.elapsedTimeSinceLastForce >= timerInfo.randomTimeInterval) {
      float angle = static_cast<float>(std::rand()) / RAND_MAX * 2 * M_PI;

      // New desired velocity based on random angle
      timerInfo.desiredVelocity =
          b2Vec2(std::cos(angle) * currentDrone.getMaxSpeed(),
                 std::sin(angle) * currentDrone.getMaxSpeed());

      // Reset the timer and generate a new random time interval for this drone
      timerInfo.elapsedTimeSinceLastForce = 0.0f;
      timerInfo.randomTimeInterval = generateRandomTimeInterval();
    }

    steer = timerInfo.desiredVelocity - currentDrone.getVelocity();
    clampMagnitude(steer, currentDrone.getMaxForce());

    b2Vec2 velocity = currentDrone.getVelocity();
    b2Vec2 position = currentDrone.getPosition();
    b2Vec2 acceleration = (force_weight_ * steer) +
                          (obstacle_avoidance_weight_ * obstacleAvoidance) +
                          neighbourAvoidance;

    velocity += acceleration;
    float speed = 0.001f + velocity.Length();
    b2Vec2 dir(velocity.x / speed, velocity.y / speed);

    // Clamp speed
    if (speed > currentDrone.getMaxSpeed()) {
      speed = currentDrone.getMaxSpeed();
    } else if (speed < 0) {
      speed = 0.001f;
    }
    velocity = b2Vec2(dir.x * speed, dir.y * speed);

    currentDrone.getBody()->SetLinearVelocity(velocity);
  }

 private:
  static float generateRandomTimeInterval() {
    return static_cast<float>(std::rand()) / RAND_MAX * 5.0f;
  }
};

auto uniform_random_walk =
    std::make_unique<swarm::UniformRandomWalkBehaviour>(10.0f, 1.0f, 1.0f);

auto uniform_random_walk_behaviour = behaviour::Registry::getInstance().add(
    "Uniform Random Walk", std::move(uniform_random_walk));
}  // namespace swarm