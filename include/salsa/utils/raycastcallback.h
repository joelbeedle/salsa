/// @file raycastcallback.h
/// @brief Defines the RayCastCallback class.
#ifndef SWARM_SIM_UTILS_RAYCASTCALLBACK_H
#define SWARM_SIM_UTILS_RAYCASTCALLBACK_H

#include <box2d/box2d.h>

#include <vector>

class Drone;  // Forward declaration

namespace swarm {

/// @class RayCastCallback
/// @brief Custom callback class for handling raycast results in the simulation.
///
/// This class extends the Box2D `b2RayCastCallback` to collect information
/// about detected drones and obstacles during a raycast operation.
class RayCastCallback : public b2RayCastCallback {
 public:
  std::vector<b2Body *>
      detectedDrones;  ///< Vector of pointers to detected drone bodies.
  std::vector<b2Vec2>
      obstaclePoints;  ///< Vector of points where obstacles were detected.

  /// @brief Report fixture method to handle the results of a raycast.
  ///
  /// This method is called for each fixture hit by the ray. It determines if
  /// the fixture is a sensor (drone sensor) or an obstacle and acts
  /// accordingly.
  ///
  /// @param fixture The fixture hit by the ray.
  /// @param point The point of initial intersection.
  /// @param normal The normal vector at the point of intersection.
  /// @param fraction The fraction along the ray at which the intersection
  /// occurs.
  /// @return The fraction to indicate the progress of the raycast. Returning -1
  /// means the ray should ignore this fixture and continue.
  float ReportFixture(b2Fixture *fixture, const b2Vec2 &point,
                      const b2Vec2 &normal, float fraction) override {
    // Keep raycasting if the fixture is a sensor, as this is just another
    // drone's sensor
    if (fixture->IsSensor()) {
      return -1;
    }
    if (fixture->GetFilterData().categoryBits != 0x0001) {
      return -1;
    }
    b2Body *body = fixture->GetBody();

    // Check the body type to differentiate between drones and obstacles
    if (body->GetType() == b2_dynamicBody) {
      return -1;
    }
    if (body->GetType() == b2_staticBody) {
      obstaclePoints.push_back(point);
    }

    return fraction;
  }
};
}  // namespace swarm

#endif  // SWARM_SIM_UTILS_RAYCASTCALLBACK_H