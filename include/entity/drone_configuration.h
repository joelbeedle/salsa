// DroneConfiguration.h
#ifndef SWARM_SIM_UTILS_DRONE_CONFIGURATION_H
#define SWARM_SIM_UTILS_DRONE_CONFIGURATION_H

#include <string>
#include <vector>
namespace swarm {
class DroneConfiguration {
 public:
  float cameraViewRange;
  float obstacleViewRange;
  float maxSpeed;
  float maxForce;
  float radius;
  float mass;
  float droneDetectionRange;

  std::string name_;
  static std::vector<DroneConfiguration *> config_registry_;

  DroneConfiguration(std::string name, float cameraView, float obstacleView,
                     float maxSpd, float maxFrc, float rad, float mss,
                     float droneDetectRange)
      : name_(name),
        cameraViewRange(cameraView),
        obstacleViewRange(obstacleView),
        maxSpeed(maxSpd),
        maxForce(maxFrc),
        radius(rad),
        mass(mss),
        droneDetectionRange(droneDetectRange) {
    config_registry_.push_back(this);
  }

  const std::string &getName() const { return name_; }

  static std::vector<std::string> getDroneConfigurationNames() {
    std::vector<std::string> names;
    for (auto config : config_registry_) {
      names.push_back(config->name_);
    }
    return names;
  }

  static DroneConfiguration *getDroneConfigurationByName(std::string name) {
    for (auto config : config_registry_) {
      if (config->name_ == name) {
        return config;
      }
    }
    return nullptr;
  }
};

}  // namespace swarm

#endif  // SWARM_SIM_UTILS_DRONE_CONFIGURATION_H