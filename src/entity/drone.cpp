// Drone.cpp
#include "salsa/entity/drone.h"

#include <cmath>
#include <valarray>

#include "salsa/utils/object_types.h"

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600

namespace swarm {
Drone::Drone(b2World *world, const b2Vec2 &position, Behaviour &behaviour,
             const DroneConfiguration &config)
    : Entity(world, position, false, config.radius, swarm::get_type<Drone>()),
      behaviour_(&behaviour),
      camera_view_range_(config.cameraViewRange),
      obstacle_view_range_(config.obstacleViewRange),
      max_speed_(config.maxSpeed),
      max_force_(config.maxForce),
      radius_(config.radius),
      mass_(config.mass),
      drone_detection_range_(config.droneDetectionRange) {
  b2CircleShape circleShape;
  circleShape.m_radius = radius_;
  b2FixtureDef fixtureDef;
  fixtureDef.shape = &circleShape;
  float area_m2 = M_PI * pow(radius_, 2);

  // Calculating the required density for Box2D
  float density_box2d = mass_ / area_m2;

  fixtureDef.density = density_box2d;
  UserData *userData = new UserData();
  userData->object = this;
  CollisionConfig c = CollisionManager::getCollisionConfig<Drone>();
  fixtureDef.filter.categoryBits = 0x0002;
  fixtureDef.filter.maskBits = 0x0001;
  fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(userData);
  // fixtureDef.filter.groupIndex = -1;
  body_->CreateFixture(&fixtureDef);

  // Create tree detecting sensor (downwards camera)
  create_fixture();

  // Initialize random starting velocity
  float angle = (rand() % 360) * (M_PI / 180.0);
  float speed = (rand() % static_cast<int>(max_speed_)) + 1;
  b2Vec2 velocity(speed * cos(angle), speed * sin(angle));
  body_->SetLinearVelocity(velocity);

  // CollisionManager::registerType(typeid(Drone), {typeid(Tree)});

  std::vector<b2Body *> obstacles;
}

Drone::~Drone() {}

void Drone::create_fixture() {
  b2CircleShape shape;
  shape.m_radius = camera_view_range_;

  b2FixtureDef fixtureDef;
  fixtureDef.shape = &shape;
  fixtureDef.isSensor = true;

  CollisionConfig config = CollisionManager::getCollisionConfig<Drone>();
  fixtureDef.filter.categoryBits = config.categoryBits;
  fixtureDef.filter.maskBits = config.maskBits;
  UserData *userData = new UserData();
  userData->object = this;

  fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(userData);

  view_sensor_ = body_->CreateFixture(&fixtureDef);
}

void Drone::updateSensorRange() {
  body_->DestroyFixture(view_sensor_);
  create_fixture();
}

void Drone::clearLists() { targets_found_.clear(); }

void Drone::update(const std::vector<std::unique_ptr<Drone>> &drones) {
  if (behaviour_) {
    behaviour_->execute(drones, *this);
  }
  b2Vec2 position = body_->GetPosition();

  body_->SetTransform(position, body_->GetAngle());
}

}  // namespace swarm