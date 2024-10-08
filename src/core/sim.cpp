#include "salsa/core/sim.h"

#include <algorithm>
#include <execution>

#include "salsa/behaviours/registry.h"
#include "salsa/utils/base_contact_listener.h"
namespace salsa {

Sim::Sim(b2World *world, const int drone_count, const int target_count,
         DroneConfiguration *config, const float border_width,
         const float border_height, const float time_limit)
    : world_(world),
      border_height_(border_height),
      border_width_(border_width),
      time_limit_(time_limit),
      drone_configuration_(config),
      num_drones_(drone_count),
      num_targets_(target_count) {
  const b2Vec2 gravity(0.0f, 0.0f);
  world_->SetGravity(gravity);

  salsa::Logger::switch_log_file("test.log");
  createBounds();
  drone_spawn_position_ = b2Vec2(border_width_ / 2, border_height_ / 2);

  createDrones(*behaviour_, *drone_configuration_, SpawnType::CIRCULAR);
}

Sim::Sim(TestConfig &config)
    : map_name_(config.map_name),
      num_drones_(config.num_drones),
      num_targets_(config.num_targets),
      drone_configuration_(DroneConfiguration::getDroneConfigurationByName(
          config.drone_config_name)),
      time_limit_(config.time_limit),
      target_type_(config.target_type),
      contact_listener_(
          BaseContactListener::getListenerByName(config.contact_listener_name)),
      test_config_(config) {
  logger::get()->info("Sim Initialised");
  is_stack_test_ = true;
  // Load map
  map_ = salsa::map::load(map_name_.c_str());
  world_ = map_.world;
  border_width_ = map_.width;
  border_height_ = map_.height;
  drone_spawn_position_ = map_.drone_spawn_point;
  b2Vec2 gravity(0.0f, 0.0f);
  world_->SetGravity(gravity);
  current_behaviour_name_ = config.behaviour_name;
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto tm = *std::localtime(&time_t);
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch()) %
                      1000;

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << "." << std::setw(3)
      << milliseconds.count() << "_" << current_behaviour_name_.c_str()
      << "/result.log";
  current_log_file_ = oss.str();
  salsa::Logger::switch_log_file(current_log_file_);

  world_->SetContactListener(contact_listener_);
  addObserver(std::shared_ptr<Logger>(&logger_, [](auto *) {}));

  auto behaviour_pointer =
      behaviour::Registry::get().behaviour(current_behaviour_name_);
  auto visitor = [&](auto &&arg) { behaviour_pointer->setParameters(arg); };
  std::visit(visitor, config.parameters);
  nlohmann::json old_message;
  std::unordered_map<std::string, behaviour::Parameter *> params =
      behaviour_pointer->getParameters();
  for (const auto &[key, value] : params) {
    old_message[key] = value->value();
  }
  old_message["behaviour"] = current_behaviour_name_;
  old_message["drone_spawn_position"] = {drone_spawn_position_.x,
                                         drone_spawn_position_.y};
  old_message["num_drones"] = num_drones_;
  old_message["num_targets"] = num_targets_;
  old_message["time_limit"] = time_limit_;
  old_message["target_type"] = target_type_;
  old_message["border_dimensions"] = {border_width_, border_height_};

  nlohmann::json message;
  message["time"] = current_time_;
  message["message"] = old_message.dump();
  message["id"] = 0;
  message["caller_type"] = "Sim";

  for (auto &observer : observers_) {
    observer->update(message);
  }

  createDrones(*behaviour_, *drone_configuration_, SpawnType::CIRCULAR);
  createTargets();
}

Sim::~Sim() {
  for (const auto &obstacle : obstacles_) {
    world_->DestroyBody(obstacle);
  }
}

void Sim::update() {
  if (current_time_ <= time_limit_ && current_time_ > 0.0) {
    num_time_steps_++;
    targets_found_this_step_.clear();
    for (const auto &drone : drones_) {
      drone->update(drones_);
      targets_found_this_step_.insert(targets_found_this_step_.end(),
                                      drone->targets_found().begin(),
                                      drone->targets_found().end());
      drone->clearLists();
      // Data logging
      b2Vec2 position = drone->position();
      b2Vec2 velocity = drone->velocity();
      if (num_time_steps_ >= log_interval_) {
        drone->notifyAll(current_time_,
                         {{"position", {position.x, position.y}},
                          {"velocity", {velocity.x, velocity.y}}});
      }
    }
    int targets_found = countFoundTargets();
    const nlohmann::json old_message = {{"targets_found", targets_found}};
    if (num_time_steps_ >= log_interval_) {
      nlohmann::json message;
      message["time"] = current_time_;
      message["message"] = old_message.dump();
      message["id"] = 0;
      message["caller_type"] = "Sim";

      for (const auto &observer : observers_) {
        observer->update(message);
      }
      num_time_steps_ = 0;
    }
  }
}

void Sim::reset() {
  current_time_ = 0.0;
  b2Vec2 gravity(0.0f, 0.0f);
  world_->SetGravity(gravity);
  drones_.clear();
  targets_.clear();
  createDrones(*behaviour_, *drone_configuration_, SpawnType::CIRCULAR);
}

void Sim::applyCurrentBehaviour()const {
  for (auto &drone : drones_) {
    drone->behaviour() = behaviour_;
  }
}

void Sim::addBehaviour(const std::string &name,
                       std::unique_ptr<salsa::Behaviour> behaviour) {
  salsa::behaviour::Registry::get().add(name, std::move(behaviour));
}

void Sim::setCurrentBehaviour(const std::string &name) {
  current_behaviour_name_ = name;
  behaviour_ = behaviour::Registry::get().behaviour(name);
  applyCurrentBehaviour();
}

void Sim::setCurrentBehaviour(Behaviour *behaviour) {
  behaviour_ = behaviour;
  applyCurrentBehaviour();
}

void Sim::createDrones(Behaviour &behaviour, DroneConfiguration &configuration,
                       SpawnType mode) {
  logger::get()->info("Creating {} drones", num_drones_);
  switch (mode) {
    case SpawnType::CIRCULAR:
      createDronesCircular(behaviour, configuration);
      break;
    case SpawnType::RANDOM:
    default:
      createDronesRandom(behaviour, configuration);
  }
  logger::get()->info("Created {} drones", drones_.size());
}

void Sim::createDronesRandom(Behaviour &behaviour, const DroneConfiguration &config) {
  for (int i = 0; i < num_drones_; i++) { constexpr float margin = 2.0f;
    float x = (rand() % static_cast<int>(border_width_ - 2 * margin)) + margin;
    float y = (rand() % static_cast<int>(border_height_ - 2 * margin)) + margin;
    drones_.push_back(
        DroneFactory::createDrone(world_, b2Vec2(x, y), behaviour, config));
  }
  for (const auto &drone : drones_) {
    drone->addObserver(std::shared_ptr<Logger>(&logger_, [](auto *) {}));
  }
}

void Sim::createDronesCircular(Behaviour &behaviour,
                               const DroneConfiguration &config) {
  // Calculate the total area needed for all drones
  float droneArea = M_PI * std::pow(config.radius, 2);
  float totalDroneArea = std::pow(num_drones_, 2) * droneArea;

  // Calculate the radius of the circle needed to fit all drones
  float requiredCircleRadius = sqrt(totalDroneArea / M_PI);

  // Adjust center of the circle to be the center of the map
  float centerX = drone_spawn_position_.x;
  float centerY = drone_spawn_position_.y;

  for (int i = 0; i < num_drones_; i++) {
    // generate random angle and radius within the required circle
    const float theta = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
    // Ensure drones fit within the required circle, leaving a margin equal to
    // the drone's radius
    const float r = sqrt(static_cast<float>(rand()) / RAND_MAX) *
              (requiredCircleRadius - config.radius);

    // Convert polar coordinates (r, theta) to Cartesian coordinates (x, y)
    const float x = centerX + r * cos(theta);
    const float y = centerY + r * sin(theta);

    drones_.push_back(
        DroneFactory::createDrone(world_, b2Vec2(x, y), behaviour, config));
  }
  int current_id = 0;
  for (const auto &drone : drones_) {
    drone->color(b2Color(0.7f, 0.5f, 0.5f));
    drone->id(current_id++);
    drone->addObserver(std::shared_ptr<Logger>(&logger_, [](auto *) {}));
  }
}

void Sim::setDroneCount(const int count) { num_drones_ = count; }

int Sim::getDroneCount()const { return num_drones_; }

int &Sim::num_drones() { return num_drones_; }
const int &Sim::num_drones() const { return num_drones_; }

void Sim::setDroneConfiguration(DroneConfiguration *configuration) {
  drone_configuration_ = configuration;
}

void Sim::updateDroneSettings()const {
  for (auto &drone : drones_) {
    drone->max_force(drone_configuration_->maxForce);
    drone->max_speed(drone_configuration_->maxSpeed);
    drone->camera_view_range(drone_configuration_->cameraViewRange);
    drone->obstacle_view_range(drone_configuration_->obstacleViewRange);
    drone->drone_detection_range(drone_configuration_->droneDetectionRange);
    drone->updateSensorRange();
  }
}

std::vector<std::unique_ptr<salsa::Drone>> &Sim::getDrones() { return drones_; }

void Sim::setDrones(std::vector<std::unique_ptr<salsa::Drone>> drones) {
  drones_ = std::move(drones);
}

template <typename... Params>
void Sim::createTargets(Params... params) {
  int id = 0;
  logger::get()->info("Creating {} targets", num_targets_);
  for (int i = 0; i < num_targets_; i++) {
    float x = (rand() % static_cast<int>(border_width_));
    float y = (rand() % static_cast<int>(border_height_));
    const b2Vec2 position(x, y);
    auto target = TargetFactory::createTarget(
        target_type_, world_, std::ref(position), id++, std::any());
    targets_.push_back(target);
  }
  logger::get()->info("Created {} targets", targets_.size());
  for (auto &target : targets_) {
    if (target) {
      target->color(b2Color(0.5f * 0.95294f, 0.5f * 0.50588f, 0.5f * 0.50588f,
                            0.5f * 0.25f));
    } else {
      std::cout << "Target is null" << std::endl;
    }
  }
  logger::get()->info("Targets created");
}

void Sim::setTargetType(const std::string &type_name) { target_type_ = type_name; }

void Sim::setTargetCount(const int count) { num_targets_ = count; }

std::vector<std::shared_ptr<Target>> &Sim::getTargets() { return targets_; }

void Sim::setContactListener(BaseContactListener &listener) {
  contact_listener_ = &listener;
  world_->SetContactListener(contact_listener_);
}

b2Vec2 &Sim::getDroneSpawnPosition() { return drone_spawn_position_; }

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

b2World *Sim::getWorld()const { return world_; }

void Sim::setWorld(b2World *world) { world_ = world; }

map::Map Sim::getMap() { return map_; }

TestConfig &Sim::test_config() { return test_config_; }

std::string &Sim::current_behaviour_name() { return current_behaviour_name_; }

const std::string &Sim::current_behaviour_name() const {
  return current_behaviour_name_;
}

DroneConfiguration *Sim::getDroneConfiguration()const {
  return drone_configuration_;
}

const DroneConfiguration *Sim::drone_configuration() const {
  return drone_configuration_;
}

std::vector<Target *> &Sim::getTargetsFoundThisStep() {
  return targets_found_this_step_;
}

int Sim::countFoundTargets() {
  return std::count_if(targets_.begin(), targets_.end(),
                       [](auto &target) { return target->isFound(); });
}

void Sim::setCurrentDroneConfiguration(DroneConfiguration &configuration) {
  drone_configuration_ = &configuration;
  updateDroneSettings();
}

void Sim::changeMap(std::string name) {
  logger::get()->info("Changing map to {}", name);
  const map::Map map = map::load(name.c_str());
  map_ = map;
  world_ = map.world;
  border_width_ = map.width;
  border_height_ = map.height;
  drone_spawn_position_ = map.drone_spawn_point;
  reset();
}

std::string &Sim::getCurrentLogFile() { return current_log_file_; }
}  // namespace salsa