// DSPBehaviour.h
#pragma once

#include <box2d/box2d.h>

#include "RayCastCallback.h"
#include "SwarmBehaviour.h"

class Drone;

struct DSPParameters {};

class DSPPoint {
 public:
  float radius = 1.0f;
  float v_max = 1.0f;
  float mass = 1.0f;
  float p = 2.0f;
  float F_max = (mass * v_max) / (1.0f / 30.0f);
  float density = (M_PI * sqrt(3)) / 1.0f;
  float area = 2000.0f * 2000.0f;
  float maxAreaCoverage = density * area;
  float numAgents = 50.0f;
  float searchAreaPerAgent = maxAreaCoverage / numAgents;
  float R = 2 * sqrt(searchAreaPerAgent / M_PI);
  float G_const = F_max * pow(R, p) * pow(2 - pow(1.5f, (1 - p)), p / (1 - p));

  b2Vec2 position;
  b2Vec2 velocity;
  b2Body *body;

  DSPPoint(b2World *world, const b2Vec2 &position) {
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position = position;
    body = world->CreateBody(&bodyDef);

    // Create Box2D fixture
    b2CircleShape circleShape;
    circleShape.m_radius = radius;

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &circleShape;
    float area_m2 = M_PI * pow(radius, 2);

    // Calculating the required density for Box2D
    float density_box2d = mass / area_m2;

    fixtureDef.density = density_box2d;

    body->CreateFixture(&fixtureDef);

    // Initialize random starting velocity
    body->SetLinearVelocity(b2Vec2(0, 0));
  }

  float gravDSPForce(b2Vec2 &position, b2Vec2 &otherPoint) {
    float distance = b2Distance(otherPoint, position);
    if (distance < 0.0001f) {  // Prevent division by zero
      distance = 0.0001f;
    }

    float forceDirection =
        (distance > R) ? 1.0f
                       : -1.0f;  // Attractive if greater than R, else repulsive

    float result = forceDirection * G_const / pow(distance, p);
    return result;
  }
};

class DSPBehaviour : public SwarmBehaviour {
 private:
  std::vector<DSPPoint *> dspPoints;
  DSPParameters params;
  float firstRun = true;

  std::unordered_map<std::string, ParameterDefinition> cleanParams = {

  };

  struct DroneInfo {
    bool isAtDSPPoint;
    b2Vec2 dspPoint;
    DSPPoint *dsp;
  };

  std::unordered_map<Drone *, DroneInfo> droneInformation;

 public:
  DSPBehaviour(const DSPParameters &params) : params(params) {}

  void execute(const std::vector<std::unique_ptr<Drone>> &drones,
               Drone &currentDrone) override;

  std::unordered_map<std::string, ParameterDefinition> getParameters()
      override {
    return cleanParams;
  }

 private:
  b2Vec2 directionTo(b2Vec2 &position, b2Vec2 &otherPoint) {
    float angle = atan2((otherPoint.y - position.y), otherPoint.x - position.x);
    b2Vec2 direction(cos(angle), sin(angle));
    return direction;
  }
};