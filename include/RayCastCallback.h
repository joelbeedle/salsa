// RayCastCallback.h
#pragma once
#include <box2d/box2d.h>

#include <vector>

class Drone;  // Forward declaration

class RayCastCallback : public b2RayCastCallback {
 public:
  std::vector<b2Body*> detectedDrones;
  std::vector<b2Vec2> obstaclePoints;

  float ReportFixture(b2Fixture* fixture, const b2Vec2& point,
                      const b2Vec2& normal, float fraction) override {
    b2Body* body = fixture->GetBody();

    // Check the body type to differentiate between drones and obstacles
    if (body->GetType() == b2_dynamicBody) {
      // Assuming drones are dynamic bodies
      detectedDrones.push_back(body);
    } else if (body->GetType() == b2_staticBody) {
      // Assuming obstacles are static bodies
      obstaclePoints.push_back(point);
    }

    // Return the fraction to continue the ray cast through all fixtures
    return -1.0f;
  }
};