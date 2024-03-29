// DroneFactory.h
#pragma once

#include <box2d/box2d.h>

#include "Drone.h"
#include "DroneConfiguration.h"

class DroneFactory {
 public:
  static Drone* createDrone(b2World* world, const b2Vec2& position,
                            SwarmBehaviour* behaviour,
                            const DroneConfiguration& config) {
    return new Drone(world, position, behaviour, config);
  }
};
