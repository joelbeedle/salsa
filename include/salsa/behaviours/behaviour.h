/// @file behaviour.h
/// @brief Contains the abstract base class for all salsa behaviours to inherit.
#ifndef SWARM_SIM_BEHAVIOURS_BEHAVIOUR_H
#define SWARM_SIM_BEHAVIOURS_BEHAVIOUR_H

#include <box2d/box2d.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "salsa/behaviours/parameter.h"
#include "salsa/utils/raycastcallback.h"
namespace salsa {

class Drone;

/// @brief Abstract base class for all salsa behaviours to inherit from.
class Behaviour {
 protected:
  /// @brief Map of parameter names to their settings.
  std::unordered_map<std::string, behaviour::Parameter *> parameters_;

 public:
  virtual ~Behaviour() = default;

  /// @brief Executes behavior logic to update the drone's velocity.
  /// All behaviors must implement this method.
  ///
  /// @param drones List of all drones in the simulation context.
  /// @param currentDrone Reference to the drone currently executing this
  /// behavior.
  virtual void execute(const std::vector<std::unique_ptr<Drone>> &drones,
                       Drone &currentDrone) = 0;

  /// @brief Retrieves a map of parameter names to their settings as described
  /// in `ParameterDefinition`. This is used in order to dynamically change
  /// behaviour parameters on the fly.
  ///
  /// @return Unordered map of parameter names to their settings.
  std::unordered_map<std::string, behaviour::Parameter *> getParameters() {
    return parameters_;
  }

  /// @brief Retrieves a map of parameter names to their current values.
  std::unordered_map<std::string, float> getParameterValues();

  static std::unordered_map<std::string, float> convertParametersToFloat(
      const std::unordered_map<std::string, behaviour::Parameter*> &parameters);

  static std::unordered_map<std::string, float> convertParametersToFloat(
      std::unordered_map<std::string, float> parameters);

  /// @brief Cleans up any resources or states specific to the behavior.
  ///
  /// @param drones List of all drones in the simulation, used for
  /// context-specific cleanup.
  virtual void clean(const std::vector<std::unique_ptr<Drone>> &drones) {}

  void setParameters(const std::unordered_map<std::string, float> &parameters);

  void setParameters(
      const std::unordered_map<std::string, salsa::behaviour::Parameter *>
          &parameters);

 protected:
  /// @brief Clamps the magnitude of a vector to a specified maximum.
  ///
  /// @param vector The vector to clamp.
  /// @param maxMagnitude The maximum magnitude allowed for the vector.
  static void clampMagnitude(b2Vec2 &vector, const float maxMagnitude);

  /// @brief Calculates a vector to avoid drones in the vicinity.
  ///
  /// @param neighbours List of neighboring drone bodies.
  /// @param currentDrone Reference to the current drone.
  /// @return A vector indicating the direction to steer to avoid the drones.
  static b2Vec2 avoidDrones(const std::vector<b2Body *> &neighbours, const Drone &currentDrone);

  /// @brief Calculates a vector to avoid nearby obstacles.
  ///
  /// @param obstaclePoints List of points representing obstacles.
  /// @param currentDrone Reference to the current drone.
  /// @return A vector indicating the direction to steer to avoid the obstacles.
  static b2Vec2 avoidObstacles(const std::vector<b2Vec2> &obstaclePoints,
                        const Drone &currentDrone);

  /// @brief Calculates a steering direction towards a specified target.
  ///
  /// @param target The target point to steer towards.
  /// @param currentDrone Reference to the current drone.
  /// @return A vector indicating the steering direction towards the target.
  static b2Vec2 steerTo(b2Vec2 target, const Drone &currentDrone);

  /// @brief Executes ray casting to detect obstacles or other elements.
  ///
  /// @param currentDrone Reference to the current drone.
  /// @param callback Raycast callback to handle detection results.
  static void performRayCasting(const Drone &currentDrone, RayCastCallback &callback);
};

// }  // namespace behaviours
}  // namespace salsa

#endif  // SWARM_SIM_BEHAVIOURS_BEHAVIOUR_H