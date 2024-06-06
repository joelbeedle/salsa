#ifndef SWARM_SIM_BEHAVIOURS_DSP_H
#define SWARM_SIM_BEHAVIOURS_DSP_H

#include <box2d/box2d.h>

#include "behaviours/behaviour.h"
#include "utils/raycastcallback.h"

class Drone;

namespace swarm_sim {
// namespace behaviours {

struct DSPParameters {};

class DSPPoint {
 public:
  float radius = 2.0f;
  float v_max = 45.0f;
  float mass = 1.0f;
  float p = 2.0f;
  float F_max = (mass * v_max) / (1.0f / 30.0f);
  // float density = (M_PI / 4);
  float density = (M_PI * sqrt(3)) / 6;
  float area = 2000.0f * 2000.0f;
  float maxAreaCoverage = density * area;
  float numAgents = 50.0f;
  float searchAreaPerAgent = maxAreaCoverage / numAgents;
  float R = (numAgents / 10.0f) * sqrt(searchAreaPerAgent / M_PI);
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

    float density_box2d = mass / area_m2;

    fixtureDef.density = density_box2d;

    body->CreateFixture(&fixtureDef);

    body->SetLinearVelocity(b2Vec2(0, 0));
  }

  float gravDSPForce(b2Vec2 &position, b2Vec2 &otherPoint) {
    float distance = b2Distance(otherPoint, position);
    if (distance < 0.0001f) {
      distance = 0.0001f;
    }

    float forceDirection =
        (distance > R) ? 1.0f
                       : -1.0f;  // Attractive if greater than R, else repulsive

    float result = forceDirection * G_const / pow(distance, p);
    return result;
  }

  void recalc(int numDrones) {
    numAgents = static_cast<float>(numDrones);
    searchAreaPerAgent = maxAreaCoverage / numAgents;
    R = (2 + (numAgents / 20.0f)) * sqrt(searchAreaPerAgent / M_PI);
    G_const = F_max * pow(R, p) * pow(2 - pow(1.5f, (1 - p)), p / (1 - p));
  }
};

class DSPBehaviour : public Behaviour {
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
    bool beginWalk = false;
    float elapsedTime = 0.0f;
    // calculated from sqrt(4000000 + 4000000) / (2 * 10.0f);
    // 141.421356237f
    float timeToWalk = 141.421356237f;
    float elapsedTimeSinceLastForce = 0.0f;
    float randomTimeInterval = 1.0f;
    b2Vec2 desiredVelocity;

    DroneInfo() : randomTimeInterval(generateRandomTimeInterval()) {}
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

  void clean(const std::vector<std::unique_ptr<Drone>> &drones) override;

 private:
  b2Vec2 directionTo(b2Vec2 &position, b2Vec2 &otherPoint) {
    float angle = atan2((otherPoint.y - position.y), otherPoint.x - position.x);
    b2Vec2 direction(cos(angle), sin(angle));
    return direction;
  }
  static float generateRandomTimeInterval() {
    return static_cast<float>(std::rand()) / RAND_MAX * 15.0f;
  }
};

}  // namespace swarm_sim
// }  // namespace swarm_sim

#endif  // SWARM_SIM_BEHAVIOURS_DSP_H