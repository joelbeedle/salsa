// Drone.h
#pragma once
#include <box2d/box2d.h>

// #include <SFML/Graphics.hpp>

#include "SwarmBehaviour.h"

class Drone {
 private:
  b2Body *body;
  SwarmBehaviour *behaviour;
  float perception;
  float maxSpeed;
  static constexpr float radius = 5.0f;

 public:
  Drone(b2World *world, const b2Vec2 &position, SwarmBehaviour *behaviour);
  ~Drone();

  void update(std::vector<Drone *> &drones);
  // void render(sf::RenderWindow &window);
  void setBehaviour(SwarmBehaviour *newBehaviour);
  std::vector<b2Body *> updateObstacles(b2World *world);

  b2Body *getBody();
  b2Vec2 getVelocity();
  b2Vec2 getPosition();
};