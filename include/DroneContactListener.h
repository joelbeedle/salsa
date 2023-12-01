#include <box2d/box2d.h>

#include <iostream>

#include "Drone.h"
#include "Tree.h"

class DroneContactListener : public b2ContactListener {
  bool getDroneAndTree(b2Contact* contact, b2Fixture*& drone,
                       b2Fixture*& tree) {
    b2Fixture* fixtureA = contact->GetFixtureA();
    b2Fixture* fixtureB = contact->GetFixtureB();

    bool sensorA = fixtureA->IsSensor();
    bool sensorB = fixtureB->IsSensor();
    if (!(sensorA ^ sensorB)) {
      return false;
    };

    // b2Body* entityA = fixtureA->GetBody();
    // b2Body* entityB = fixtureB->GetBody();

    if (sensorA) {  // fixtureB must be a done
      tree = fixtureA;
      drone = fixtureB;
    } else {  // fixtureA must be a drone
      tree = fixtureB;
      drone = fixtureA;
    }

    return true;
  }
  void BeginContact(b2Contact* contact) override {
    b2Fixture* drone;
    b2Fixture* tree;

    if (getDroneAndTree(contact, drone, tree)) {
      Tree* thisTree = reinterpret_cast<Tree*>(tree->GetUserData().pointer);
      thisTree->setMapped(true);
    }
  }

  void EndContact(b2Contact* contact) override {
    // Handle end of contact if needed
  }
};