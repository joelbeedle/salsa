#ifndef SWARM_SIM_CORE_SIM_H
#define SWARM_SIM_CORE_SIM_H

#include <box2d/box2d.h>

#include <variant>

#include "behaviours/behaviour.h"
#include "behaviours/registry.h"
#include "core/test_queue.h"
#include "entity/drone.h"
#include "entity/drone_configuration.h"
#include "entity/drone_factory.h"
#include "entity/target.h"
#include "utils/base_contact_listener.h"

namespace swarm {

struct DroneParameters {
  float cameraViewRange;
  float obstacleViewRange;
  float droneDetectionRange;
  float maxSpeed;
  float maxForce;
  float mass;
  float radius;
};

class Sim {
 private:
  // Current map
  map::Map map_;
  // Box2D world pointer
  b2World* world_;
  // Contact listener for Box2D collisions
  swarm::BaseContactListener* contact_listener_;

  // Simulation configurations and parameters
  swarm::TestConfig test_config_;
  float border_height_;
  float border_width_;
  b2Vec2 drone_spawn_position_;
  float time_limit_ = -1.0f;
  float current_time_ = 0.0f;
  bool is_stack_test_ = false;

  // Drone management
  std::unordered_map<std::string, DroneParameters> all_drone_parameters_;
  std::unordered_map<std::string, swarm::DroneConfiguration>
      all_drone_configurations_;
  swarm::DroneConfiguration* drone_configuration_;
  std::vector<std::unique_ptr<swarm::Drone>> drones_;
  int num_drones_;
  float max_speed_;
  float max_force_;

  // Target management
  std::vector<swarm::Target*> targets_;
  float num_targets_;

  // Obstacles in the environment
  std::vector<b2Body*> obstacles_;
  float obstacle_view_range_;

  // Behaviour management
  swarm::Behaviour* behaviour_;
  std::string current_behaviour_name_;
  float camera_view_range_;

  // Visualization flags
  bool draw_visual_range_ = false;
  bool draw_targets_ = false;
  bool draw_drones_ = false;

  // Logging
  Logger& logger_ = Logger::getInstance();

  // Private methods for internal use
  void createBounds();
  void applyCurrentBehaviour();
  void createDronesCircular(Behaviour& behaviour,
                            DroneConfiguration& configuration);
  void createDronesRandom(Behaviour& behaviour,
                          DroneConfiguration& configuration);

 public:
  // Constructors and Destructor
  Sim(b2World* world, int drone_count, int target_count,
      DroneConfiguration* config, float border_width, float border_height,
      float time_limit);
  Sim(swarm::TestConfig& config);
  ~Sim();

  enum class SpawnType { CIRCULAR, RANDOM };

  // Simulation control
  void update();
  void reset();

  // Behaviour functions
  void addBehaviour(const std::string& name,
                    std::unique_ptr<swarm::Behaviour> behaviour);
  void setCurrentBehaviour(Behaviour* behaviour);
  void setCurrentBehaviour(const std::string& name);

  // Drone functions
  void createDrones(Behaviour& behaviour, DroneConfiguration& configuration,
                    SpawnType mode);
  void setDroneCount(int count);
  int getDroneCount();
  int& num_drones();
  const int& num_drones() const;
  void setDroneConfiguration(DroneConfiguration* configuration);
  void updateDroneSettings();
  std::vector<std::unique_ptr<swarm::Drone>>& getDrones();
  void setDrones(std::vector<std::unique_ptr<swarm::Drone>> drones);

  // Target functions
  void setTargetCount(int count);

  // Box2D functions
  void setContactListener(BaseContactListener& listener);

  // Getters and Setters for properties
  float& world_height();
  const float& world_height() const;
  float& world_width();
  const float& world_width() const;
  float& current_time();
  const float& current_time() const;
  float& time_limit();
  b2World* getWorld();
  void setWorld(b2World* world);
  map::Map getMap();
  swarm::TestConfig& test_config();
  std::string& current_behaviour_name();
  const std::string& current_behaviour_name() const;
  DroneConfiguration* getDroneConfiguration();
  const DroneConfiguration* drone_configuration() const;
  void setCurrentDroneConfiguration(DroneConfiguration& configuration);
};

class SimBuilder {
 private:
  b2World* world_;
  Behaviour* behaviour_;
  int drone_count_ = 0;
  int target_count_ = 0;
  float world_height_ = 0;
  float world_width_ = 0;
  float time_limit_ = -1.0f;
  DroneConfiguration* config_;
  BaseContactListener* contact_listener_;

 public:
  SimBuilder& setWorld(b2World* world) {
    world_ = world;
    return *this;
  }

  SimBuilder& setDroneCount(int count) {
    drone_count_ = count;
    return *this;
  }

  SimBuilder& setTargetCount(int count) {
    target_count_ = count;
    return *this;
  }

  SimBuilder& setContactListener(BaseContactListener& listener) {
    contact_listener_ = &listener;
    world_->SetContactListener(&listener);
    return *this;
  }

  SimBuilder& setDroneConfiguration(DroneConfiguration* config) {
    config_ = config;
    return *this;
  }

  SimBuilder& setWorldHeight(float height) {
    world_height_ = height;
    return *this;
  }

  SimBuilder& setWorldWidth(float width) {
    world_width_ = width;
    return *this;
  }

  SimBuilder& setTimeLimit(float time_limit) {
    time_limit_ = time_limit;
    return *this;
  }

  Sim* build() {
    std::cout << "Building sim with:\nDrone count: " << drone_count_
              << "\nTarget Count: " << target_count_
              << "\nWorld Height: " << world_height_
              << "\nWorld Width: " << world_width_
              << "\nConfig: " << (config_ != nullptr) << std::endl;
    return new Sim(world_, drone_count_, target_count_, config_, world_width_,
                   world_height_, time_limit_);
  }
};

}  // namespace swarm

#endif  // SWARM_SIM_CORE_SIM_H
