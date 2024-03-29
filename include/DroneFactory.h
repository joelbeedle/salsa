// DroneFactory.h
#pragma once

#include <box2d/box2d.h>

#include <memory>

#include "Drone.h"
#include "DroneConfiguration.h"

class DroneFactory {
 public:
  static std::unique_ptr<Drone> createDrone(b2World* world,
                                            const b2Vec2& position,
                                            SwarmBehaviour& behaviour,
                                            const DroneConfiguration& config) {
    return std::make_unique<Drone>(world, position, behaviour, config);
  }
};
