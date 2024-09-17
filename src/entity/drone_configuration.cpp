#include <utility>

#include "salsa/entity/drone_configuration.h"

using namespace salsa;
std::vector<DroneConfiguration *> DroneConfiguration::config_registry_;

DroneConfiguration::DroneConfiguration(std::string name, const float cameraView,
                                       const float obstacleView, const float maxSpd,
                                       const float maxFrc, const float rad, const float mss,
                                       const float droneDetectRange)
    : name_(std::move(name)),
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
  names.reserve(config_registry_.size());
for (const auto config : config_registry_) {
    names.push_back(config->name_);
  }
  return names;
}

DroneConfiguration *DroneConfiguration::getDroneConfigurationByName(
    const std::string& name) {
  for (const auto config : config_registry_) {
    if (config->name_ == name) {
      return config;
    }
  }
  return nullptr;
}
