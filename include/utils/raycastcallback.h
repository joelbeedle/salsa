// RayCastCallback.h
#ifndef SWARM_SIM_UTILS_RAYCASTCALLBACK_H
#define SWARM_SIM_UTILS_RAYCASTCALLBACK_H

#include <box2d/box2d.h>

#include <vector>

class Drone;  // Forward declaration

namespace swarm_sim {
class RayCastCallback : public b2RayCastCallback {
 public:
  std::vector<b2Body*> detectedDrones;
  std::vector<b2Vec2> obstaclePoints;

  float ReportFixture(b2Fixture* fixture, const b2Vec2& point,
                      const b2Vec2& normal, float fraction) override {
    // Keep raycasting if the fixture is a sensor, as this is just another
    // drone's sensor
    if (fixture->IsSensor()) {
      return -1;
    }
    b2Body* body = fixture->GetBody();

    // Check the body type to differentiate between drones and obstacles
    if (body->GetType() == b2_dynamicBody) {
      detectedDrones.push_back(body);
    } else if (body->GetType() == b2_staticBody) {
      obstaclePoints.push_back(point);
    }

    return fraction;
  }
};
}  // namespace swarm_sim

#endif  // SWARM_SIM_UTILS_RAYCASTCALLBACK_H