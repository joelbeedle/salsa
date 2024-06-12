#ifndef SWARM_SIM_CORE_SIM_H
#define SWARM_SIM_CORE_SIM_H

#include <box2d/box2d.h>

#include <variant>

#include "behaviours/behaviour.h"
#include "behaviours/registry.h"
#include "core/test_stack.h"
#include "drones/drone.h"
#include "drones/drone_factory.h"
#include "target.h"
#include "utils/base_contact_listener.h"
#include "utils/drone_configuration.h"

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
  b2World *world_;

  float border_height_;
  float border_width_;

  swarm::Behaviour *behaviour_;
  std::string current_behaviour_name_;

  // Setup drone parameters and configurations
  std::unordered_map<std::string, DroneParameters> all_drone_parameters_;
  std::unordered_map<std::string, swarm::DroneConfiguration>
      all_drone_configurations_;

  swarm::DroneConfiguration *drone_configuration_;
  DroneParameters *drone_parameters_;

  std::vector<std::unique_ptr<swarm::Drone>> drones_;
  std::vector<swarm::Target *> targets_;
  std::vector<b2Body *> obstacles_;

  swarm::BaseContactListener *contact_listener_;

  float obstacle_view_range_;
  float camera_view_range_;
  float max_speed_;
  float max_force_;

  float num_drones_;
  float num_targets_;
  float is_stack_test_ = false;
  float time_limit_ = -1.0f;
  float current_time_ = 0.0f;

  bool draw_visual_range_ = false;
  bool draw_targets_ = false;
  bool draw_drones = false;

 public:
  Sim(b2World *world, int drone_count, int target_count,
      DroneConfiguration *config, float border_width, float border_height,
      float time_limit);
  Sim(b2World *world, swarm::TestConfig &config);
  ~Sim();
  void run();
  void init();
  void update();
  void reset();
  void cleanup();
  void updateImGui();
  // Behaviour functions
  void addBehaviour(const std::string &name,
                    std::unique_ptr<swarm::Behaviour> behaviour);
  void applyCurrentBehaviour();

  // Drone functions
  void createDrones(Behaviour &behaviour, DroneConfiguration &configuration);
  void setDroneCount(int count);
  void setDroneConfiguration(DroneConfiguration *configuration);
  void updateDroneSettings();

  // Target functions
  void setTargetCount(int count);

  // Box2D functions
  void setContactListener(BaseContactListener &listener);

  float &world_height() { return border_height_; }
  const float &world_height() const { return border_height_; }

  float &world_width() { return border_width_; }
  const float &world_width() const { return border_width_; }

  float &current_time() { return current_time_; }
  const float &current_time() const { return current_time_; }

  float &time_limit() { return time_limit_; }
  b2World *getWorld() { return world_; }

  std::string getBehaviourName() { return current_behaviour_name_; }

  std::string &current_behaviour_name() { return current_behaviour_name_; }
  const std::string &current_behaviour_name() const {
    return current_behaviour_name_;
  }
  void setBehaviour(Behaviour *behaviour) { behaviour_ = behaviour; }
  Behaviour *getBehaviour() { return behaviour_; }
  Behaviour *behaviour() { return behaviour_; }
  const Behaviour *behaviour() const { return behaviour_; }
  void setCurrentBehaviour(const std::string &name) {
    behaviour_ = behaviour::Registry::getInstance().getBehaviour(name);
    current_behaviour_name_ = name;
  }
  int getDroneCount() { return num_drones_; }

  DroneConfiguration *getDroneConfiguration() { return drone_configuration_; }
  void setCurrentDroneConfiguration(DroneConfiguration &configuration) {
    drone_configuration_ = &configuration;
  }

  DroneConfiguration *drone_configuration() { return drone_configuration_; }
  const DroneConfiguration *drone_configuration() const {
    return drone_configuration_;
  }

 private:
  void createBounds();
};

class SimBuilder {
 private:
  b2World *world_;
  Behaviour *behaviour_;
  int drone_count_ = 0;
  int target_count_ = 0;
  float world_height_ = 0;
  float world_width_ = 0;
  float time_limit_ = -1.0f;
  DroneConfiguration *config_;
  BaseContactListener *contact_listener_;

 public:
  SimBuilder &setWorld(b2World *world) {
    world_ = world;
    return *this;
  }

  SimBuilder &setDroneCount(int count) {
    drone_count_ = count;
    return *this;
  }

  SimBuilder &setTargetCount(int count) {
    target_count_ = count;
    return *this;
  }

  SimBuilder &setContactListener(BaseContactListener &listener) {
    contact_listener_ = &listener;
    world_->SetContactListener(&listener);
    return *this;
  }

  SimBuilder &setDroneConfiguration(DroneConfiguration *config) {
    config_ = config;
    return *this;
  }

  SimBuilder &setWorldHeight(float height) {
    world_height_ = height;
    return *this;
  }

  SimBuilder &setWorldWidth(float width) {
    world_width_ = width;
    return *this;
  }

  SimBuilder &setTimeLimit(float time_limit) {
    time_limit_ = time_limit;
    return *this;
  }

  Sim *build() {
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
