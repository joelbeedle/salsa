#include "PheremoneBehaviour.h"

#include "Drone.h"

void PheremoneBehaviour::execute(std::vector<Drone *> &drones,
                                 Drone *currentDrone) {
  // Lay a pheremone at the Drone's current position
  layPheremone(currentDrone->getPosition());

  // Update pheremones (decay)
  updatePheremones();
  b2Vec2 steering(0, 0);
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