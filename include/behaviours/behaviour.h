#ifndef SWARM_SIM_BEHAVIOURS_BEHAVIOUR_H
#define SWARM_SIM_BEHAVIOURS_BEHAVIOUR_H

#include <box2d/box2d.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "utils/raycastcallback.h"
namespace swarm {

// Forward declaration for Drone class
class Drone;
// namespace behaviours {

/// @brief Structure for behavior parameters with adjustable settings.
struct ParameterDefinition {
  float* value;     ///< Pointer to the value of the parameter.
  float min_value;  ///< Minimum allowable value for the parameter.
  float max_value;  ///< Maximum allowable value for the parameter.
};

/// @brief Abstract base class for all swarm behaviours to inherit from.
class Behaviour {
 public:
  virtual ~Behaviour() = default;

  /// @brief Executes behavior logic to update the drone's velocity.
  /// All behaviors must implement this method.
  /// @param drones List of all drones in the simulation context.
  /// @param currentDrone Reference to the drone currently executing this
  /// behavior.
  virtual void execute(const std::vector<std::unique_ptr<Drone>>& drones,
                       Drone& currentDrone) = 0;

  /// @brief Retrieves a map of parameter names to their settings as described
  /// in `ParameterDefinition`. This is used in order to dynamically change
  /// behaviour parameters on the fly.
  /// @return Unordered map of parameter names to their settings.
  virtual std::unordered_map<std::string, ParameterDefinition>
  getParameters() = 0;

  /// @brief Cleans up any resources or states specific to the behavior.
  /// @param drones List of all drones in the simulation, used for
  /// context-specific cleanup.
  virtual void clean(const std::vector<std::unique_ptr<Drone>>& drones) {}

 protected:
  /// @brief Clamps the magnitude of a vector to a specified maximum.
  /// @param vector The vector to clamp.
  /// @param maxMagnitude The maximum magnitude allowed for the vector.
  void clampMagnitude(b2Vec2& vector, const float maxMagnitude);

  /// @brief Calculates a vector to avoid drones in the vicinity.
  /// @param neighbours List of neighboring drone bodies.
  /// @param currentDrone Reference to the current drone.
  /// @return A vector indicating the direction to steer to avoid the drones.
  b2Vec2 avoidDrones(std::vector<b2Body*>& neighbours, Drone& currentDrone);

  /// @brief Calculates a vector to avoid nearby obstacles.
  /// @param obstaclePoints List of points representing obstacles.
  /// @param currentDrone Reference to the current drone.
  /// @return A vector indicating the direction to steer to avoid the obstacles.
  b2Vec2 avoidObstacles(std::vector<b2Vec2>& obstaclePoints,
                        Drone& currentDrone);

  /// @brief Calculates a steering direction towards a specified target.
  /// @param target The target point to steer towards.
  /// @param currentDrone Reference to the current drone.
  /// @return A vector indicating the steering direction towards the target.
  b2Vec2 steerTo(b2Vec2 target, Drone& currentDrone);

  /// @brief Executes ray casting to detect obstacles or other elements.
  /// @param currentDrone Reference to the current drone.
  /// @param callback Raycast callback to handle detection results.
  void performRayCasting(Drone& currentDrone, RayCastCallback& callback);
};

// }  // namespace behaviours
}  // namespace swarm

#endif  // SWARM_SIM_BEHAVIOURS_BEHAVIOUR_H