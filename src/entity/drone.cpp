// Drone.cpp
#include "entity/drone.h"

#include <cmath>
#include <valarray>

#include "utils/object_types.h"

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600

namespace swarm {
Drone::Drone(b2World *world, const b2Vec2 &position, Behaviour &behaviour,
             const DroneConfiguration &config)
    : Entity(world, position, false, config.radius),
      behaviour(&behaviour),
      cameraViewRange(config.cameraViewRange),
      obstacleViewRange(config.obstacleViewRange),
      maxSpeed(config.maxSpeed),
      maxForce(config.maxForce),
      radius_(config.radius),
      mass(config.mass),
      droneDetectionRange(config.droneDetectionRange) {
  id_prefix = 'D';
  b2CircleShape circleShape;
  circleShape.m_radius = radius_;
  b2FixtureDef fixtureDef;
  fixtureDef.shape = &circleShape;
  float area_m2 = M_PI * pow(radius_, 2);

  // Calculating the required density for Box2D
  float density_box2d = mass / area_m2;

  fixtureDef.density = density_box2d;
  UserData *userData = new UserData();
  userData->type = ObjectType::Drone;
  userData->object = this;

  fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(userData);
  body_->CreateFixture(&fixtureDef);

  // Create tree detecting sensor (downwards camera)
  create_fixture();

  // Initialize random starting velocity
  float angle = (rand() % 360) * (M_PI / 180.0);
  float speed = (rand() % static_cast<int>(maxSpeed)) + 1;
  b2Vec2 velocity(speed * cos(angle), speed * sin(angle));
  body_->SetLinearVelocity(velocity);

  // CollisionManager::registerType(typeid(Drone), {typeid(Tree)});

  std::vector<b2Body *> obstacles;
}

Drone::~Drone() {}

void Drone::create_fixture() {
  b2CircleShape shape;
  shape.m_radius = cameraViewRange;

  b2FixtureDef fixtureDef;
  fixtureDef.shape = &shape;
  fixtureDef.isSensor = true;

  CollisionConfig config = CollisionManager::getCollisionConfig(typeid(Drone));
  fixtureDef.filter.categoryBits = config.categoryBits;
  fixtureDef.filter.maskBits = config.maskBits;
  UserData *userData = new UserData();
  userData->type = ObjectType::Drone;
  userData->object = this;

  fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(userData);

  viewSensor = body_->CreateFixture(&fixtureDef);
}

void Drone::updateSensorRange() {
  body_->DestroyFixture(viewSensor);
  create_fixture();
}

void Drone::clearLists() { this->foundDiseasedTreePositions.clear(); }

void Drone::update(const std::vector<std::unique_ptr<Drone>> &drones) {
  if (behaviour) {
    behaviour->execute(drones, *this);
  }
  b2Vec2 position = body_->GetPosition();

  body_->SetTransform(position, body_->GetAngle());
}

}  // namespace swarm