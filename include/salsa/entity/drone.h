/// @file drone.h
/// @brief Contains the `Drone` class, which represents a single drone in the
/// simulation.
#ifndef SWARM_SIM_DRONES_DRONE_H
#define SWARM_SIM_DRONES_DRONE_H

#include <box2d/box2d.h>

#include <memory>

#include "drone_configuration.h"
#include "entity.h"
#include "salsa/behaviours/behaviour.h"
#include "salsa/utils/collision_manager.h"
#include "target.h"
namespace salsa {

/// @class Drone
/// @brief Represents a drone entity in the simulation environment.
///
/// Inherits from Entity and includes additional properties and functionalities
/// specific to drones, such as behaviors, sensor range, and physical
/// properties.
class Drone : public Entity {
 private:
 private:
  std::vector<Target *>
      targets_found_;       ///< List of targets detected by the drone
  b2Fixture *view_sensor_{};  ///< Sensor fixture for target detection
  Behaviour *behaviour_;    ///< Current behavior governing the drone's actions

  /// @name Phsyical and Sensor attributes
  ///@{
  float camera_view_range_;
  float obstacle_view_range_;
  float drone_detection_range_;
  float max_speed_;
  float max_force_;
  float radius_;
  float mass_;
  ///@}

 public:
  /// @brief Constructor to create a drone with specified parameters.
  /// @param world Pointer to the b2World where the drone operates.
  /// @param position Initial position of the drone.
  /// @param behaviour Behaviour assigned to control the drone.
  /// @param config Configuration settings of the drone.
  Drone(b2World *world, const b2Vec2 &position, Behaviour &behaviour,
        const DroneConfiguration &config);
  /// @brief Destructor for Drone.
  virtual ~Drone();

  void create_fixture();

  /// @brief Updates drone's state and behavior execution for a simulation step.
  /// @param drones List of all drones in the simulation for interaction.
  void update(const std::vector<std::unique_ptr<Drone>> &drones);

  /// @brief Clears the list of targets found by the drone.
  void clearLists();

  /// @brief Updates the view sensor of the drone.
  void updateSensorRange();

  /// @name Identity-oriented getters and setters for reference types
  ///@{
  Behaviour *&behaviour() { return behaviour_; }
  const Behaviour *behaviour() const { return behaviour_; }

  b2Body *body() const { return body_; }

  b2Fixture *&view_sensor() { return view_sensor_; }
  const b2Fixture *view_sensor() const { return view_sensor_; }
  ///@}

  /// @name Value-oriented getters and setters for fundamental types
  ///@{
  float camera_view_range() const { return camera_view_range_; }
  void camera_view_range(float new_range) { camera_view_range_ = new_range; }

  float drone_detection_range() const { return drone_detection_range_; }
  void drone_detection_range(float new_range) {
    drone_detection_range_ = new_range;
  }

  float obstacle_view_range() const { return obstacle_view_range_; }
  void obstacle_view_range(float new_range) {
    obstacle_view_range_ = new_range;
  }

  float max_speed() const { return max_speed_; }
  void max_speed(float new_speed) { max_speed_ = new_speed; }

  float max_force() const { return max_force_; }
  void max_force(float new_force) { max_force_ = new_force; }

  const std::vector<Target *> &targets_found() const { return targets_found_; }
  void addTargetFound(Target *target) { targets_found_.push_back(target); }
  ///@}

  /// @name Physics properties exposed directly
  ///@{
  b2Vec2 velocity() const { return body_->GetLinearVelocity(); }
  void clear_lists() { targets_found_.clear(); }
  ///@}
};

}  // namespace salsa

#endif  // SWARM_SIM_DRONES_DRONE_H