#include <box2d/b2_math.h>
#include <box2d/box2d.h>
#include <salsa/salsa.h>

#include <map>
#include <memory>
#include <unordered_map>
namespace swarm {
class PheromoneBehaviour : public Behaviour {
 private:
  struct Pheromone {
    b2Vec2 position;
    float intensity;
  };

  std::map<int, Pheromone> pheromones;
  int pheromoneCount = 0;
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
               Drone &currentDrone) override {
    // Perform ray casting to detect nearby drones and obstacles
    RayCastCallback callback;
    performRayCasting(currentDrone, callback);

    std::vector<b2Vec2> obstaclePoints = callback.obstaclePoints;
    std::vector<b2Body *> neighbours = callback.detectedDrones;

    layPheromone(currentDrone.getPosition());

    updatePheromones();
    b2Vec2 steering = obstacle_avoidance_weight_ *
                      avoidObstacles(obstaclePoints, currentDrone);

    b2Vec2 avoidanceSteering(0, 0);
    int32 count = 0;

    for (auto &pair : pheromones) {
      Pheromone &Pheromone = pair.second;
      float distance =
          b2Distance(currentDrone.getPosition(), Pheromone.position);

      if (distance < currentDrone.getObstacleViewRange() && distance > 0) {
        b2Vec2 awayFromPheromone =
            currentDrone.getPosition() - Pheromone.position;
        awayFromPheromone.Normalize();

        awayFromPheromone *= (1.0f / (distance)) * Pheromone.intensity;

        avoidanceSteering += awayFromPheromone;
        count++;
      }
    }
  }

 private:
  void updatePheromones() {
    for (auto it = pheromones.begin(); it != pheromones.end();) {
      it->second.intensity -= decay_rate_;
      if (it->second.intensity <= 0) {
        it = pheromones.erase(it);
      } else {
        ++it;
      }
    }
  }
  void layPheromone(const b2Vec2 &position) {
    Pheromone pheromone = {position, 500.0f};
    pheromones[pheromoneCount++] = pheromone;
  }
};

auto p = std::make_unique<swarm::PheromoneBehaviour>(0.5f, 1.0f);

auto pheromone =
    behaviour::Registry::getInstance().add("Pheromone Avoidance", std::move(p));
}  // namespace swarm