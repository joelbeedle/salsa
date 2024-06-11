#ifndef SWARM_SIM_CORE_SIM_H
#define SWARM_SIM_CORE_SIM_H

#include <box2d/box2d.h>

#include "behaviours/behaviour.h"
#include "behaviours/registry.h"
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

  swarm::BaseContactListener *contactListener_;

  float obstacle_view_range_;
  float camera_view_range_;
  float max_speed_;
  float max_force_;

  float num_drones_;
  float num_targets_;

  bool draw_visual_range_ = false;
  bool draw_targets_ = false;
  bool draw_drones = false;

 public:
  Sim(b2World *world, int drone_count, int target_count,
      DroneConfiguration *config);
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

  b2World *getWorld() { return world_; }

  std::string &current_behaviour_name() { return current_behaviour_name_; }
  const std::string &current_behaviour_name() const {
    return current_behaviour_name_;
  }

  Behaviour *behaviour() { return behaviour_; }
  const Behaviour *behaviour() const { return behaviour_; }
  void setCurrentBehaviour(const std::string &name) {
    behaviour_ = behaviour::Registry::getInstance().getBehaviour(name);
    current_behaviour_name_ = name;
  }

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
  int drone_count_;
  int target_count_;
  DroneConfiguration *config_;

 public:
  SimBuilder(b2World *world, int drone_count, int target_count,
             DroneConfiguration *config)
      : world_(world),
        drone_count_(drone_count),
        target_count_(target_count),
        config_(config) {}

  SimBuilder &setBehaviour(const std::string &name) {
    behaviour_ = behaviour::Registry::getInstance().getBehaviour(name);
    return *this;
  }

  SimBuilder &setDroneConfiguration(DroneConfiguration *config) {
    config_ = config;
    return *this;
  }

  Sim build() { return Sim(world_, drone_count_, target_count_, config_); }
};

}  // namespace swarm

#endif  // SWARM_SIM_CORE_SIM_H