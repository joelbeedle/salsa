// Drone.cpp
#include "Drone.h"

#include <valarray>

#include "ObjectTypes.h"

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600

Drone::Drone(b2World *world, const b2Vec2 &position, SwarmBehaviour *behaviour,
             float viewRange, float maxSpeed, float maxForce, float radius)
    : behaviour(behaviour),
      viewRange(viewRange),
      maxSpeed(maxSpeed),
      maxForce(maxForce),
      radius(radius) {
  // Create Box2D body
  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  bodyDef.position = position;
  body = world->CreateBody(&bodyDef);

  // Create Box2D fixture
  b2CircleShape circleShape;
  circleShape.m_radius = radius;

  b2FixtureDef fixtureDef;
  fixtureDef.shape = &circleShape;
  fixtureDef.density = 1.0f;
  UserData *userData = new UserData();
  userData->type = ObjectType::Drone;  // or ObjectType::Tree for a tree
  userData->drone = this;              // or userData->tree = this for a tree

  fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(userData);
  body->CreateFixture(&fixtureDef);

  b2CircleShape sCircleShape;
  sCircleShape.m_radius = viewRange;

  b2FixtureDef sFixtureDef;
  sFixtureDef.shape = &sCircleShape;
  sFixtureDef.isSensor = true;
  sFixtureDef.filter.categoryBits = 0x0001;
  sFixtureDef.filter.maskBits = 0x0002;
  sFixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(this);

  viewSensor = body->CreateFixture(&sFixtureDef);

  std::vector<b2Body *> obstacles;
}

Drone::~Drone() {
  if (body) {
    b2World *world = body->GetWorld();
    if (world) {
      world->DestroyBody(body);
    }
  }
  if (behaviour) {
    delete behaviour;
  }
}

void Drone::updateSensorRange() {
  b2CircleShape sCircleShape;
  sCircleShape.m_radius = viewRange;

  b2FixtureDef sFixtureDef;
  sFixtureDef.shape = &sCircleShape;
  sFixtureDef.isSensor = true;
  sFixtureDef.filter.categoryBits = 0x0001;
  sFixtureDef.filter.maskBits = 0x0002;
  sFixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(this);

  body->DestroyFixture(viewSensor);
  viewSensor = body->CreateFixture(&sFixtureDef);
}

void Drone::update(std::vector<Drone *> &drones) {
  if (behaviour) {
    behaviour->execute(drones, this);
  }

  b2Vec2 position = body->GetPosition();

  body->SetTransform(position, body->GetAngle());
}