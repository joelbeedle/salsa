#include "core/sim.h"

#include "behaviours/registry.h"

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

  createDrones(*behaviour_, *drone_configuration_, SpawnType::CIRCULAR);
}

Sim::Sim(TestConfig &config)
    : world_(config.world),
      num_drones_(config.num_drones),
      num_targets_(config.num_targets),
      drone_configuration_(config.drone_config),
      time_limit_(config.time_limit),
      test_config_(config) {
  is_stack_test_ = true;
  b2Vec2 gravity(0.0f, 0.0f);
  world_->SetGravity(gravity);
  logger_ = std::make_shared<Logger>("test.log");
  current_behaviour_name_ = config.behaviour_name;
  auto behaviour_pointer =
      behaviour::Registry::getInstance().getBehaviour(current_behaviour_name_);
  auto visitor = [&](auto &&arg) { behaviour_pointer->setParameters(arg); };
  std::visit(visitor, config.parameters);
  createDrones(*behaviour_, *drone_configuration_, SpawnType::CIRCULAR);
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
  createDrones(*behaviour_, *drone_configuration_, SpawnType::CIRCULAR);
}

void Sim::applyCurrentBehaviour() {
  for (auto &drone : drones_) {
    drone->setBehaviour(*behaviour_);
  }
}

void Sim::addBehaviour(const std::string &name,
                       std::unique_ptr<swarm::Behaviour> behaviour) {
  swarm::behaviour::Registry::getInstance().add(name, std::move(behaviour));
}

void Sim::setCurrentBehaviour(const std::string &name) {
  current_behaviour_name_ = name;
  behaviour_ = behaviour::Registry::getInstance().getBehaviour(name);
  applyCurrentBehaviour();
}

// Maybe unsafe
void Sim::setCurrentBehaviour(Behaviour *behaviour) {
  behaviour_ = behaviour;
  applyCurrentBehaviour();
}

void Sim::createDrones(Behaviour &behaviour, DroneConfiguration &configuration,
                       SpawnType mode) {
  switch (mode) {
    case SpawnType::CIRCULAR:
      createDronesCircular(behaviour, configuration);
      break;
    case SpawnType::RANDOM:
      createDronesRandom(behaviour, configuration);
    default:
      createDronesRandom(behaviour, configuration);
  }
}

void Sim::createDronesRandom(Behaviour &behaviour, DroneConfiguration &config) {
  const float margin = 2.0f;
  for (int i = 0; i < num_drones_; i++) {
    float x = (rand() % static_cast<int>(border_width_ - 2 * margin)) + margin;
    float y = (rand() % static_cast<int>(border_height_ - 2 * margin)) + margin;
    drones_.push_back(
        DroneFactory::createDrone(world_, b2Vec2(x, y), behaviour, config));
  }
  for (auto &drone : drones_) {
    drone->addObserver(logger_);
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
  for (auto &drone : drones_) {
    drone->addObserver(logger_);
  }
}

void Sim::setDroneCount(int count) { num_drones_ = count; }

int Sim::getDroneCount() { return num_drones_; }

void Sim::setDroneConfiguration(DroneConfiguration *configuration) {
  drone_configuration_ = configuration;
}

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

std::vector<std::unique_ptr<swarm::Drone>> &Sim::getDrones() { return drones_; }

void Sim::setDrones(std::vector<std::unique_ptr<swarm::Drone>> drones) {
  drones_ = std::move(drones);
}

void Sim::setTargetCount(int count) { num_targets_ = count; }

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

float &Sim::world_height() { return border_height_; }
const float &Sim::world_height() const { return border_height_; }

float &Sim::world_width() { return border_width_; }
const float &Sim::world_width() const { return border_width_; }

float &Sim::current_time() { return current_time_; }
const float &Sim::current_time() const { return current_time_; }

float &Sim::time_limit() { return time_limit_; }

b2World *Sim::getWorld() { return world_; }

void Sim::setWorld(b2World *world) { world_ = world; }

TestConfig &Sim::test_config() { return test_config_; }

std::string &Sim::current_behaviour_name() { return current_behaviour_name_; }

const std::string &Sim::current_behaviour_name() const {
  return current_behaviour_name_;
}

DroneConfiguration *Sim::getDroneConfiguration() {
  return drone_configuration_;
}

const DroneConfiguration *Sim::drone_configuration() const {
  return drone_configuration_;
}

void Sim::setCurrentDroneConfiguration(DroneConfiguration &configuration) {
  drone_configuration_ = &configuration;
  updateDroneSettings();
}

}  // namespace swarm