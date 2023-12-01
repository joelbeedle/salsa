// Drone.h
#pragma once
#include <box2d/box2d.h>

// #include <SFML/Graphics.hpp>

#include "SwarmBehaviour.h"

class Drone {
 private:
  b2Body *body;
  b2Fixture *viewSensor;  // The view range sensor
  SwarmBehaviour *behaviour;
  float viewRange;
  float maxSpeed;
  float maxForce;
  float radius;

 public:
  Drone(b2World *world, const b2Vec2 &position, SwarmBehaviour *behaviour,
        float viewRange, float maxSpeed, float maxForce, float radius);
  ~Drone();

  void update(std::vector<Drone *> &drones);
  void updateSensorRange();

  // Accessors and Mutators
  void setBehaviour(SwarmBehaviour *newBehaviour) { behaviour = newBehaviour; }

  b2Body *getBody() { return body; }
  b2Vec2 getVelocity() { return body->GetLinearVelocity(); }
  b2Vec2 getPosition() { return body->GetPosition(); }

  float getViewRange() { return viewRange; }
  void setViewRange(float newRange) { viewRange = newRange; }

  float getMaxSpeed() { return maxSpeed; }
  void setMaxSpeed(float newSpeed) { maxSpeed = newSpeed; }

  float getMaxForce() { return maxForce; }
  void setMaxForce(float newForce) { maxForce = newForce; }

  float getRadius() { return radius; }
};