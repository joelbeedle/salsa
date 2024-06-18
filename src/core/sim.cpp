#include "core/sim.h"

namespace swarm {

Sim::Sim(b2World *world, int drone_count, int target_count,
         DroneConfiguration *configuration, float border_width,
         float border_height, float time_limit)
    : world_(world),
      num_drones_(drone_count),
      num_targets_(target_count),
      drone_configuration_(configuration),
      border_width_(border_width),
      border_height_(border_height),
      time_limit_(time_limit) {
  b2Vec2 gravity(0.0f, 0.0f);
  world_->SetGravity(gravity);
  createBounds();

  createDrones(*behaviour_, *drone_configuration_);
}

Sim::Sim(b2World *world, TestConfig &config)
    : world_(config.world),
      num_drones_(config.num_drones),
      num_targets_(config.num_targets),
      drone_configuration_(config.drone_config),
      time_limit_(config.time_limit) {
  is_stack_test_ = true;
  b2Vec2 gravity(0.0f, 0.0f);
  world_->SetGravity(gravity);
  // createBounds();
  current_behaviour_name_ = config.behaviour_name;
  auto behaviour_pointer =
      behaviour::Registry::getInstance().getBehaviour(current_behaviour_name_);
  auto visitor = [&](auto &&arg) { behaviour_pointer->setParameters(arg); };
  std::visit(visitor, config.parameters);
  createDronesCircular(*behaviour_, *drone_configuration_);
}

Sim::~Sim() {
  for (auto &obstacle : obstacles_) {
    world_->DestroyBody(obstacle);
  }
}

void Sim::update() {
  for (auto &drone : drones_) {
    drone->update(drones_);
  }
}

void Sim::reset() {
  current_time_ = 0.0;
  drones_.clear();
  createDrones(*behaviour_, *drone_configuration_);
}

void Sim::createDrones(Behaviour &behaviour,
                       DroneConfiguration &configuration) {
  const float margin = 2.0f;  // Define a margin to prevent spawning exactly
                              // at the border or outside
  for (int i = 0; i < num_drones_; i++) {
    float x = (rand() % static_cast<int>(4000 - 2 * margin)) + margin;
    float y = (rand() % static_cast<int>(4000 - 2 * margin)) + margin;
    drones_.push_back(DroneFactory::createDrone(world_, b2Vec2(x, y), behaviour,
                                                configuration));
  }
}

void Sim::createDronesCircular(Behaviour &behaviour,
                               DroneConfiguration &config) {
  // Calculate the total area needed for all drones
  float droneArea = M_PI * std::pow(config.radius, 2);
  float totalDroneArea = num_drones_ * droneArea;

  // Calculate the radius of the circle needed to fit all drones
  float requiredCircleRadius = sqrt(totalDroneArea / M_PI);

  // Adjust center of the circle to be the center of the map
  // TODO: Make this the point of the drone spawn point.
  float centerX = 1000 / 2.0f;
  float centerY = 1000 / 2.0f;

  for (int i = 0; i < num_drones_; i++) {
    // generate random angle and radius within the required circle
    float theta = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
    // Ensure drones fit within the required circle, leaving a margin equal to
    // the drone's radius
    float r = sqrt(static_cast<float>(rand()) / RAND_MAX) *
              (requiredCircleRadius - config.radius);

    // Convert polar coordinates (r, theta) to Cartesian coordinates (x, y)
    float x = centerX + r * cos(theta);
    float y = centerY + r * sin(theta);

    drones_.push_back(
        DroneFactory::createDrone(world_, b2Vec2(x, y), behaviour, config));
  }
}

void Sim::setDroneCount(int count) { num_drones_ = count; }

void Sim::updateDroneSettings() {
  for (auto &drone : drones_) {
    drone->setMaxForce(drone_configuration_->maxForce);
    drone->setMaxSpeed(drone_configuration_->maxSpeed);
    drone->setViewRange(drone_configuration_->cameraViewRange);
    drone->setObstacleViewRange(drone_configuration_->obstacleViewRange);
    drone->setDroneDetectionRange(drone_configuration_->droneDetectionRange);
    drone->updateSensorRange();
  }
}

void Sim::setDroneConfiguration(DroneConfiguration *configuration) {
  drone_configuration_ = configuration;
}

void Sim::setTargetCount(int count) { num_targets_ = count; }

void Sim::addBehaviour(const std::string &name,
                       std::unique_ptr<swarm::Behaviour> behaviour) {
  swarm::behaviour::Registry::getInstance().add(name, std::move(behaviour));
}

void Sim::applyCurrentBehaviour() {
  for (auto &drone : drones_) {
    drone->setBehaviour(*behaviour_);
  }
}

void Sim::setContactListener(BaseContactListener &listener) {
  contact_listener_ = &listener;
  world_->SetContactListener(contact_listener_);
}

void Sim::createBounds() {
  // Define the ground body.
  b2BodyDef groundBodyDef;
  groundBodyDef.position.Set(0.0f, 0.0f);

  b2Body *groundBody = world_->CreateBody(&groundBodyDef);

  b2EdgeShape groundBox;

  // bottom
  groundBox.SetTwoSided(b2Vec2(0.0f, 0.0f), b2Vec2(border_width_, 0.0f));
  groundBody->CreateFixture(&groundBox, 0.0f);

  // top
  groundBox.SetTwoSided(b2Vec2(0.0f, border_height_),
                        b2Vec2(border_width_, border_height_));
  groundBody->CreateFixture(&groundBox, 0.0f);

  // left
  groundBox.SetTwoSided(b2Vec2(0.0f, 0.0f), b2Vec2(0.0f, border_height_));
  groundBody->CreateFixture(&groundBox, 0.0f);

  // right
  groundBox.SetTwoSided(b2Vec2(border_width_, 0.0f),
                        b2Vec2(border_width_, border_height_));
  groundBody->CreateFixture(&groundBox, 0.0f);

  obstacles_.push_back(groundBody);
}

}  // namespace swarm