#include "behaviours/pheromone_avoidance.h"

#include <map>
#include <memory>
#include <unordered_map>

#include "drones/drone.h"

void PheremoneBehaviour::execute(
    const std::vector<std::unique_ptr<Drone>> &drones, Drone &currentDrone) {
  // Perform ray casting to detect nearby drones and obstacles
  RayCastCallback callback;
  performRayCasting(currentDrone, callback);

  std::vector<b2Vec2> obstaclePoints = callback.obstaclePoints;
  std::vector<b2Body *> neighbours = callback.detectedDrones;

  layPheremone(currentDrone.getPosition());

  updatePheremones();
  b2Vec2 steering = params.obstacleAvoidanceWeight *
                    avoidObstacles(obstaclePoints, currentDrone);

  b2Vec2 avoidanceSteering(0, 0);
  int32 count = 0;

  for (auto &pair : pheremones) {
    Pheremone &pheremone = pair.second;
    float distance = b2Distance(currentDrone.getPosition(), pheremone.position);

    if (distance < currentDrone.getObstacleViewRange() && distance > 0) {
      b2Vec2 awayFromPheremone =
          currentDrone.getPosition() - pheremone.position;
      awayFromPheremone.Normalize();

      awayFromPheremone *= (1.0f / (distance)) * pheremone.intensity;

      avoidanceSteering += awayFromPheremone;
      count++;
    }
  }

  if (count > 0) {
    avoidanceSteering.x /= count;
    avoidanceSteering.y /= count;
    avoidanceSteering.Normalize();
    avoidanceSteering *= currentDrone.getMaxSpeed();

    steering += avoidanceSteering - currentDrone.getVelocity();
    clampMagnitude(steering, currentDrone.getMaxForce());
  }

  b2Vec2 acceleration =
      steering + (params.obstacleAvoidanceWeight *
                  avoidObstacles(obstaclePoints, currentDrone));
  b2Vec2 velocity = currentDrone.getVelocity();
  b2Vec2 position = currentDrone.getPosition();

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
  acceleration.SetZero();
}

void PheremoneBehaviour::layPheremone(const b2Vec2 &position) {
  Pheremone pheremone = {position, 500.0f};
  pheremones[pheremoneCount++] = pheremone;
}

void PheremoneBehaviour::updatePheremones() {
  for (auto it = pheremones.begin(); it != pheremones.end();) {
    it->second.intensity -= params.decayRate;
    if (it->second.intensity <= 0) {
      it = pheremones.erase(it);
    } else {
      ++it;
    }
  }
}