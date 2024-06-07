#ifndef SWARM_SIM_UTILS_DRONE_CONTACT_LISTENER_H
#define SWARM_SIM_UTILS_DRONE_CONTACT_LISTENER_H

#include <box2d/box2d.h>

#include <iostream>

#include "drones/drone.h"
#include "tree.h"
#include "utils/object_types.h"

namespace swarm {
class DroneContactListener : public b2ContactListener {
  bool getDroneAndTree(b2Contact *contact, b2Fixture *&drone,
                       b2Fixture *&tree) {
    b2Fixture *fixtureA = contact->GetFixtureA();
    b2Fixture *fixtureB = contact->GetFixtureB();

    bool sensorA = fixtureA->IsSensor();
    bool sensorB = fixtureB->IsSensor();
    if (!(sensorA ^ sensorB)) {
      return false;
    };

    if (sensorB) {  // fixtureB must be a done
      tree = fixtureA;
      drone = fixtureB;
    } else {  // fixtureA must be a drone
      tree = fixtureB;
      drone = fixtureA;
    }

    return true;
  }
  void BeginContact(b2Contact *contact) override {
    b2Fixture *fixtureA = contact->GetFixtureA();
    b2Fixture *fixtureB = contact->GetFixtureB();
    Tree *tree;
    Drone *drone;

    // Check if one of the fixtures is a sensor and the other is a tree
    if (fixtureA->IsSensor() &&
        fixtureB->GetFilterData().categoryBits == 0x0002) {
      // fixtureA is the drone's sensor, fixtureB is the tree
      UserData *userData =
          reinterpret_cast<UserData *>(fixtureB->GetUserData().pointer);
      tree = userData->tree;
      UserData *droneData =
          reinterpret_cast<UserData *>(fixtureA->GetUserData().pointer);
      drone = droneData->drone;
      drone->foundDiseasedTree(tree);
      tree->setMapped(true);
      tree->addNumMapped();

    } else if (fixtureB->IsSensor() &&
               fixtureA->GetFilterData().categoryBits == 0x0002) {
      // fixtureB is the drone's sensor, fixtureA is the tree
      UserData *userData =
          reinterpret_cast<UserData *>(fixtureA->GetUserData().pointer);
      tree = userData->tree;
      UserData *droneData =
          reinterpret_cast<UserData *>(fixtureB->GetUserData().pointer);
      drone = droneData->drone;

      drone->foundDiseasedTree(tree);
      tree->setMapped(true);
      tree->addNumMapped();
    }
  }

  void EndContact(b2Contact *contact) override {
    b2Fixture *fixtureA = contact->GetFixtureA();
    b2Fixture *fixtureB = contact->GetFixtureB();
    Tree *tree = nullptr;

    if (fixtureB->IsSensor() &&
        fixtureA->GetFilterData().categoryBits == 0x0002) {
      UserData *userData =
          reinterpret_cast<UserData *>(fixtureA->GetUserData().pointer);
      tree = userData->tree;
    } else if (fixtureA->IsSensor() &&
               fixtureB->GetFilterData().categoryBits == 0x0002) {
      UserData *userData =
          reinterpret_cast<UserData *>(fixtureB->GetUserData().pointer);
      tree = userData->tree;
    }

    if (tree) {
      tree->resetMapping();
    }
  }
};

}  // namespace swarm

#endif  // SWARM_SIM_UTILS_DRONE_CONTACT_LISTENER_H