#include <box2d/box2d.h>
#include <salsa/salsa.h>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
namespace salsa {
class UniformRandomWalkBehaviour final : public Behaviour {
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

    DroneTimerInfo() : randomTimeInterval(generateRandomTimeInterval()),desiredVelocity() {}
  };

  std::unordered_map<Drone *, DroneTimerInfo> droneTimers;

 public:
  UniformRandomWalkBehaviour(const float maxMagnitude, const float forceWeight,
                             const float obstacleAvoidanceWeight)
      : max_magnitude_(maxMagnitude, 0.0f, 20.0f),
        force_weight_(forceWeight, 0.0f, 20.0f),
        obstacle_avoidance_weight_(obstacleAvoidanceWeight, 0.0f, 3.0f) {
    parameters_["Max Magnitude"] = &max_magnitude_;
    parameters_["Force Weight"] = &force_weight_;
    parameters_["Obstacle Avoidance Weight"] = &obstacle_avoidance_weight_;
    std::srand(std::time(nullptr));
  }

  ~UniformRandomWalkBehaviour() override = default;

  void execute(const std::vector<std::unique_ptr<Drone>> &drones,
               Drone &currentDrone) override {
    if (!droneTimers.contains(&currentDrone)  ) {
      droneTimers[&currentDrone] = DroneTimerInfo();
      droneTimers[&currentDrone].desiredVelocity = currentDrone.velocity();
    }
    RayCastCallback callback;
    performRayCasting(currentDrone, callback);

    std::vector<b2Vec2> obstaclePoints = callback.obstaclePoints;
    std::vector<b2Body *> neighbours = callback.detectedDrones;

    DroneTimerInfo &timerInfo = droneTimers[&currentDrone];
    timerInfo.elapsedTimeSinceLastForce += delta_time_;
    auto  force = b2Vec2(0, 0);
    auto  steer = b2Vec2(0, 0);
    const b2Vec2 obstacleAvoidance = avoidObstacles(obstaclePoints, currentDrone);
    const b2Vec2 neighbourAvoidance = avoidDrones(neighbours, currentDrone);

    // Check if it's time to apply a new random force
    if (timerInfo.elapsedTimeSinceLastForce >= timerInfo.randomTimeInterval) {
      const float angle = static_cast<float>(std::rand()) / RAND_MAX * 2 * M_PI;

      // New desired velocity based on random angle
      timerInfo.desiredVelocity =
          b2Vec2(std::cos(angle) * currentDrone.max_speed(),
                 std::sin(angle) * currentDrone.max_speed());

      // Reset the timer and generate a new random time interval for this drone
      timerInfo.elapsedTimeSinceLastForce = 0.0f;
      timerInfo.randomTimeInterval = generateRandomTimeInterval();
    }

    steer = timerInfo.desiredVelocity - currentDrone.velocity();
    clampMagnitude(steer, currentDrone.max_force());

    b2Vec2 velocity = currentDrone.velocity();
    b2Vec2 position = currentDrone.position();
    const b2Vec2 acceleration = (force_weight_ * steer) +
                          (obstacle_avoidance_weight_ * obstacleAvoidance) +
                          neighbourAvoidance;

    velocity += acceleration;
    float speed = 0.001f + velocity.Length();
    const b2Vec2 dir(velocity.x / speed, velocity.y / speed);

    // Clamp speed
    if (speed > currentDrone.max_speed()) {
      speed = currentDrone.max_speed();
    } else if (speed < 0) {
      speed = 0.001f;
    }
    velocity = b2Vec2(dir.x * speed, dir.y * speed);

    currentDrone.body()->SetLinearVelocity(velocity);
  }

 private:
  static float generateRandomTimeInterval() {
    return static_cast<float>(std::rand()) / RAND_MAX * 5.0f;
  }
};

auto uniform_random_walk =
    std::make_unique<salsa::UniformRandomWalkBehaviour>(10.0f, 1.0f, 1.0f);

auto uniform_random_walk_behaviour = behaviour::Registry::get().add(
    "Uniform Random Walk", std::move(uniform_random_walk));
}  // namespace salsa