#include "salsa/entity/drone_configuration.h"

using namespace swarm;
std::vector<DroneConfiguration *> DroneConfiguration::config_registry_;

DroneConfiguration::DroneConfiguration(std::string name, float cameraView,
                                       float obstacleView, float maxSpd,
                                       float maxFrc, float rad, float mss,
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

std::vector<std::string> DroneConfiguration::getDroneConfigurationNames() {
  std::vector<std::string> names;
  for (auto config : config_registry_) {
    names.push_back(config->name_);
  }
  return names;
}

DroneConfiguration *DroneConfiguration::getDroneConfigurationByName(
    std::string name) {
  for (auto config : config_registry_) {
    if (config->name_ == name) {
      return config;
    }
  }
  return nullptr;
}
