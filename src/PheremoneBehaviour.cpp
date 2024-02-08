#include "PheremoneBehaviour.h"

#include "Drone.h"

void PheremoneBehaviour::execute(std::vector<Drone *> &drones,
                                 Drone *currentDrone) {
  // Perform ray casting to detect nearby drones and obstacles
  RayCastCallback callback;
  performRayCasting(currentDrone, callback);

  // Separating drones and obstacles from callback results
  std::vector<b2Vec2> obstaclePoints = callback.obstaclePoints;

  // Lay a pheremone at the Drone's current position
  layPheremone(currentDrone->getPosition());

  // Update pheremones (decay)
  updatePheremones();
  b2Vec2 steering = params.obstacleAvoidanceWeight *
                    avoidObstacles(obstaclePoints, currentDrone);
  ;
  // Find the strongest nearby pheremone
  Pheremone *strongestNearby = nullptr;
  float maxIntensity = 0.0f;
  for (auto &pair : pheremones) {
    Pheremone &pheremone = pair.second;
    float distance =
        b2Distance(currentDrone->getPosition(), pheremone.position);
    if (distance < currentDrone->getViewRange() &&
        pheremone.intensity > maxIntensity) {
      strongestNearby = &pheremone;
      maxIntensity = pheremone.intensity;
    }
  }

  // Steer towards the strongest pheremone
  if (strongestNearby != nullptr) {
    b2Vec2 desired = strongestNearby->position - currentDrone->getPosition();
    desired.Normalize();
    desired *= 10.0f;  // change to maxSpeed
    steering = desired - currentDrone->getVelocity();
    clampMagnitude(steering, 0.3f);
  }

  currentDrone->getBody()->ApplyForceToCenter(steering, true);
}

void PheremoneBehaviour::performRayCasting(Drone *currentDrone,
                                           RayCastCallback &callback) {
  // Define the ray casting range and angle
  float rayRange = currentDrone->getViewRange();
  ;
  float deltaAngle = 15.0f;  // dividing the circle into segments

  for (float angle = 0; angle < 360; angle += deltaAngle) {
    b2Vec2 start = currentDrone->getPosition();
    b2Vec2 end = start + rayRange * b2Vec2(cosf(angle * (b2_pi / 180.0f)),
                                           sinf(angle * (b2_pi / 180.0f)));

    currentDrone->getBody()->GetWorld()->RayCast(&callback, start, end);
  }
}

b2Vec2 PheremoneBehaviour::avoidObstacles(std::vector<b2Vec2> &obstaclePoints,
                                          Drone *currentDrone) {
  b2Vec2 steering(0, 0);
  int32 count = 0;

  for (auto &point : obstaclePoints) {
    float distance = b2Distance(currentDrone->getPosition(), point);
    if (distance < currentDrone->getViewRange() && distance > 0) {
      b2Vec2 diff = currentDrone->getPosition() - point;
      // Weighted by the inverse distance
      diff.Normalize();
      diff.x /= distance;
      diff.y /= distance;
      steering += diff;
      count++;
    }
  }

  if (count > 0) {
    steering.x /= count;
    steering.y /= count;
    steering.Normalize();
    steering *= currentDrone->getMaxSpeed();
    steering -= currentDrone->getVelocity();
    clampMagnitude(steering, currentDrone->getMaxForce());
  }

  return steering;
}

void PheremoneBehaviour::layPheremone(const b2Vec2 &position) {
  Pheremone pheremone = {position, 1.0f};
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