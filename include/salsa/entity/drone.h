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
namespace swarm {
class Drone : public Entity {
 private:
  std::vector<Target *> targets_found_;
  b2Fixture *viewSensor;  ///< Sensor used for detecting targets
  Behaviour *behaviour;   ///< Current swarm `Behaviour` of the drone

  float cameraViewRange;
  float obstacleViewRange;
  float droneDetectionRange;
  float maxSpeed;
  float maxForce;
  float radius_;
  float mass;

 public:
  Drone(b2World *world, const b2Vec2 &position, Behaviour &behaviour,
        const DroneConfiguration &config);
  virtual ~Drone();
  Drone() = default;

  void create_fixture();

  /// @brief Updates one step of the simulation for this drone, where the drone
  /// runs its behaviours' `execute` function, which tells the drone what move
  /// to make in this step.
  ///
  /// @param drones The list of all drones in the simulation
  void update(const std::vector<std::unique_ptr<Drone>> &drones);
  void updateSensorRange();

  // Accessors and Mutators
  void setBehaviour(Behaviour &newBehaviour) { behaviour = &newBehaviour; }
  Behaviour *getBehaviour() { return behaviour; }

  b2Body *getBody() { return body_; }
  b2Vec2 getVelocity() { return body_->GetLinearVelocity(); }
  b2Vec2 getPosition() { return body_->GetPosition(); }

  float getViewRange() { return cameraViewRange; }
  void setViewRange(float newRange) { cameraViewRange = newRange; }

  float getDroneDetectionRange() { return droneDetectionRange; }
  void setDroneDetectionRange(float newRange) {
    droneDetectionRange = newRange;
  }

  float getObstacleViewRange() { return obstacleViewRange; }
  void setObstacleViewRange(float newRange) { obstacleViewRange = newRange; }

  float getMaxSpeed() { return maxSpeed; }
  void setMaxSpeed(float newSpeed) { maxSpeed = newSpeed; }

  float getMaxForce() { return maxForce; }
  void setMaxForce(float newForce) { maxForce = newForce; }

  b2Fixture *getViewSensor() { return viewSensor; }

  float getRadius() { return radius_; }

  void addTargetFound(Target *target) { targets_found_.push_back(target); }

  std::vector<Target *> &getTargetsFound() { return targets_found_; }

  void clearLists();
};

}  // namespace swarm

#endif  // SWARM_SIM_DRONES_DRONE_H