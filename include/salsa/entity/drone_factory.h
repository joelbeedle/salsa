// DroneFactory.h
#ifndef SWARM_SIM_DRONES_DRONE_FACTORY_H
#define SWARM_SIM_DRONES_DRONE_FACTORY_H

#include <box2d/box2d.h>

#include <memory>

#include "drone.h"
#include "drone_configuration.h"

namespace swarm {
class DroneFactory {
 public:
  static std::unique_ptr<Drone> createDrone(b2World *world,
                                            const b2Vec2 &position,
                                            Behaviour &behaviour,
                                            const DroneConfiguration &config) {
    return std::make_unique<Drone>(world, position, behaviour, config);
  }
};

}  // namespace swarm

#endif  // SWARM_SIM_DRONES_DRONE_FACTORY_H