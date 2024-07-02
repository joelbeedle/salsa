/// @file drone_factory.h
/// @brief Contains factory to construct drones
#ifndef SWARM_SIM_DRONES_DRONE_FACTORY_H
#define SWARM_SIM_DRONES_DRONE_FACTORY_H

#include <box2d/box2d.h>

#include <memory>

#include "drone.h"
#include "drone_configuration.h"

namespace swarm {
/// @brief Factory to create drones
class DroneFactory {
 public:
  /// @brief Creates a drone with the given parameters
  /// @param world The Box2D world in which the drone will exist
  /// @param position The position to create the drone in.
  /// @param behaviour The initial behaviour the drone exhibits.
  /// @param config The configuration settings for the drone
  /// @return a unique pointer to the created drone
  static std::unique_ptr<Drone> createDrone(b2World *world,
                                            const b2Vec2 &position,
                                            Behaviour &behaviour,
                                            const DroneConfiguration &config) {
    return std::make_unique<Drone>(world, position, behaviour, config);
  }
};

}  // namespace swarm

#endif  // SWARM_SIM_DRONES_DRONE_FACTORY_H