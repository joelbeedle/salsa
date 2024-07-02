/// @file drone_configuration.h
/// @brief Contains the `DroneConfiguration` class, which represents a
/// configuration
#ifndef SWARM_SIM_UTILS_DRONE_CONFIGURATION_H
#define SWARM_SIM_UTILS_DRONE_CONFIGURATION_H

#include <string>
#include <vector>
namespace salsa {

/// @brief Class representing the configuration parameters for a drone.
class DroneConfiguration {
 private:
  /// Registry of all drone configurations created.
  static std::vector<DroneConfiguration *> config_registry_;
  std::string name_;  ///< Unique name identifier for the drone configuration.

 public:
  float cameraViewRange;  ///< Range within which the drone's camera can detect
                          ///< objects.
  float obstacleViewRange;    ///< Range within which the drone can detect
                              ///< obstacles.
  float maxSpeed;             ///< Maximum speed the drone can achieve.
  float maxForce;             ///< Maximum steering force the drone can apply.
  float radius;               ///< Collision radius of the drone.
  float mass;                 ///< Mass of the drone, affecting its dynamics.
  float droneDetectionRange;  ///< Range within which the drone can detect other
                              ///< drones.

  /// @brief Constructs a new Drone Configuration and adds it to the registry.
  /// @param name Name of the drone configuration.
  /// @param cameraView Range of the camera view.
  /// @param obstacleView Range of the obstacle detection.
  /// @param maxSpd Maximum speed of the drone.
  /// @param maxFrc Maximum force the drone can exert.
  /// @param rad Radius of the drone.
  /// @param mss Mass of the drone.
  /// @param droneDetectRange Detection range for other drones.
  DroneConfiguration(std::string name, float cameraView, float obstacleView,
                     float maxSpd, float maxFrc, float rad, float mss,
                     float droneDetectRange);

  /// @brief Returns a list of names of all registered drone configurations.
  /// @return Vector of names of drone configurations.
  static std::vector<std::string> getDroneConfigurationNames();

  /// @brief Fetches a drone configuration by its name.
  /// @param name Name of the drone configuration to retrieve.
  /// @return Pointer to the DroneConfiguration, or nullptr if not found.
  static DroneConfiguration *getDroneConfigurationByName(std::string name);

  const std::string &name() const { return name_; }
};

}  // namespace salsa

#endif  // SWARM_SIM_UTILS_DRONE_CONFIGURATION_H