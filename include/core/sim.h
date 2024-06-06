#ifndef SWARM_SIM_CORE_SIM_H
#define SWARM_SIM_CORE_SIM_H

#include <stack>

#include "behaviours/behaviour.h"
#include "behaviours/flocking.h"
#include "behaviours/pheromone_avoidance.h"
#include "behaviours/registry.h"
#include "behaviours/uniform_random_walk.h"
#include "utils/drone_contact_listener.h"

namespace swarm_sim {

struct DroneParameters {
  float cameraViewRange;
  float obstacleViewRange;
  float droneDetectionRange;
  float maxSpeed;
  float maxForce;
  float mass;
  float radius;
};

class SwarmSim {
 private:
  Behaviour* behaviour_;
  std::string current_behaviour_name_;
  DroneContactListener contact_listener_;

  // Setup drone parameters and configurations
  std::unordered_map<std::string, DroneParameters> all_drone_parameters_;
  std::unordered_map<std::string, DroneConfiguration> all_drone_configurations_;

  DroneConfiguration* drone_configuration_;
  DroneParameters* drone_parameters_;

  std::vector<std::unique_ptr<Drone>> drones_;

  float obstacle_view_range_;
  float camera_view_range_;
  float max_speed_;
  float max_force_;

  float num_drones_;

 public:
  SwarmSim() {}

  void AddBehaviour(const std::string& name,
                    std::unique_ptr<Behaviour> behaviour) {
    behaviour::Registry::getInstance().add(name, std::move(behaviour));
  }

  void Start() {}

  // Accessors and Mutators

  DroneConfiguration* drone_configuration() { return drone_configuration_; }
  const DroneConfiguration* drone_configuration() const {
    return drone_configuration_;
  }

  DroneParameters* drone_parameters() { return drone_parameters_; }
  const DroneParameters* drone_parameters() const { return drone_parameters_; }
};

}  // namespace swarm_sim

#endif  // SWARM_SIM_CORE_SIM_H