#include "PheremoneBehaviour.h"

#include "Drone.h"

void PheremoneBehaviour::execute(
    const std::vector<std::unique_ptr<Drone>> &drones, Drone &currentDrone) {
  // Perform ray casting to detect nearby drones and obstacles
  RayCastCallback callback;
  performRayCasting(currentDrone, callback);

  // Separating drones and obstacles from callback results
  std::vector<b2Vec2> obstaclePoints = callback.obstaclePoints;
  std::vector<b2Body *> neighbours = callback.detectedDrones;

  // Lay a pheremone at the Drone's current position
  layPheremone(currentDrone.getPosition());

  // Update pheremones (decay)
  updatePheremones();
  // Initial obstacle avoidance steering
  b2Vec2 steering = params.obstacleAvoidanceWeight *
                    avoidObstacles(obstaclePoints, currentDrone);

  // Vector to accumulate avoidance forces from all nearby pheromones
  b2Vec2 avoidanceSteering(0, 0);
  int32 count = 0;

  for (auto &pair : pheremones) {
    Pheremone &pheremone = pair.second;
    float distance = b2Distance(currentDrone.getPosition(), pheremone.position);

    if (distance < currentDrone.getObstacleViewRange() && distance > 0) {
      // Calculate a vector pointing away from the pheremone
      b2Vec2 awayFromPheremone =
          currentDrone.getPosition() - pheremone.position;
      awayFromPheremone.Normalize();

      // Optionally, weight this vector by the inverse of distance or intensity
      // of the pheremone This makes the drone steer more strongly away from
      // closer or more intense pheremones
      awayFromPheremone *= (1.0f / (distance)) * pheremone.intensity;

      avoidanceSteering += awayFromPheremone;
      count++;
    }
  }

  // Average the avoidance steering if any pheremones were found
  if (count > 0) {
    avoidanceSteering.x /= count;
    avoidanceSteering.y /= count;
    avoidanceSteering.Normalize();
    avoidanceSteering *= currentDrone.getMaxSpeed();

    // Combine with the original steering vector
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
  acceleration.SetZero();  // TODO: Find out implications of acceleration not
                           // being transient
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