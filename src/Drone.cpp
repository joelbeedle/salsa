// Drone.cpp
#include "Drone.h"

#include <cmath>
#include <valarray>

#include "ObjectTypes.h"

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600

Drone::Drone(b2World *world, const b2Vec2 &position, SwarmBehaviour *behaviour,
             float viewRange, float obstacleViewRange, float maxSpeed,
             float maxForce, float radius, float mass)
    : behaviour(behaviour),
      viewRange(viewRange),
      obstacleViewRange(obstacleViewRange),
      maxSpeed(maxSpeed),
      maxForce(maxForce),
      radius(radius),
      mass(mass) {
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
  float area_m2 = M_PI * pow(radius, 2);

  // Calculating the required density for Box2D
  float density_box2d = mass / area_m2;

  fixtureDef.density = density_box2d;
  UserData *userData = new UserData();
  userData->type = ObjectType::Drone;  // or ObjectType::Tree for a tree
  userData->drone = this;              // or userData->tree = this for a tree

  fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(userData);
  body->CreateFixture(&fixtureDef);

  // Create tree detecting sensor (downwards camera)
  b2CircleShape sCircleShape;
  sCircleShape.m_radius = viewRange;

  b2FixtureDef sFixtureDef;
  sFixtureDef.shape = &sCircleShape;
  sFixtureDef.isSensor = true;
  sFixtureDef.filter.categoryBits = 0x0001;
  sFixtureDef.filter.maskBits = 0x0002;
  userData->drone = this;
  sFixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(userData);

  viewSensor = body->CreateFixture(&sFixtureDef);

  // Initialize random starting velocity
  float angle = (rand() % 360) * (M_PI / 180.0);
  float speed = (rand() % static_cast<int>(maxSpeed)) + 1;
  b2Vec2 velocity(speed * cos(angle), speed * sin(angle));
  body->SetLinearVelocity(velocity);

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
  UserData *userData = new UserData();
  userData->type = ObjectType::Drone;  // or ObjectType::Tree for a tree
  userData->drone = this;
  sFixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(userData);

  body->DestroyFixture(viewSensor);
  viewSensor = body->CreateFixture(&sFixtureDef);
}

void Drone::clearLists() {
  this->foundDiseasedTrees.clear();
  this->foundDiseasedTreePositions.clear();
  this->foundTrees.clear();
}

void Drone::update(std::vector<Drone *> &drones) {
  if (behaviour) {
    behaviour->execute(drones, this);
  }
  // Clear found trees for this cycle?
  // this->foundDiseasedTrees.clear();
  // this->foundDiseasedTreePositions.clear();
  // this->foundTrees.clear();

  b2Vec2 position = body->GetPosition();

  body->SetTransform(position, body->GetAngle());
}

void Drone::foundDiseasedTree(Tree *tree) {
  foundDiseasedTrees.push_back(tree);
  b2Vec2 *position = new b2Vec2(tree->getBody()->GetPosition());
  foundDiseasedTreePositions.push_back(position);
}

void Drone::foundTree(Tree *tree) { foundTrees.push_back(tree); }