#include <iostream>
#include <valarray>

#include "behaviours/flocking.h"
#include "box2d/box2d.h"
#include "drones/drone.h"
#include "utils/raycastcallback.h"

void FlockingBehaviour::execute(
    const std::vector<std::unique_ptr<Drone>> &drones, Drone &currentDrone) {
  // Using ray casting to find neighbours and obstacles
  RayCastCallback callback;
  performRayCasting(currentDrone, callback);
  std::vector<b2Body *> bodies;
  for (auto &drone : drones) {
    bodies.push_back(drone.get()->getBody());
  }

  std::vector<b2Body *> neighbours = bodies;
  std::vector<b2Vec2> obstaclePoints = callback.obstaclePoints;

  b2Vec2 alignment = align(neighbours, currentDrone);
  b2Vec2 separation = separate(neighbours, currentDrone);
  b2Vec2 cohesion = cohere(neighbours, currentDrone);
  b2Vec2 obstacleAvoidance = avoidObstacles(obstaclePoints, currentDrone);

  b2Vec2 acceleration = (params.alignmentWeight * alignment) +
                        (params.separationWeight * separation) +
                        (params.cohesionWeight * cohesion) +
                        (params.obstacleAvoidanceWeight * obstacleAvoidance);
  b2Vec2 velocity = currentDrone.getVelocity();
  b2Vec2 position = currentDrone.getPosition();

  velocity += acceleration;
  float speed = 0.001f + velocity.Length();
  b2Vec2 dir(velocity.x / speed, velocity.y / speed);

  // Clamp speed
  if (speed > currentDrone.getMaxSpeed()) {
    speed = currentDrone.getMaxSpeed();
  } else if (speed < 0) {
    speed = 0.001f;
  }
  velocity = b2Vec2(dir.x * speed, dir.y * speed);

  currentDrone.getBody()->SetLinearVelocity(velocity);
  acceleration.SetZero();
}

b2Vec2 FlockingBehaviour::align(std::vector<b2Body *> &drones,
                                Drone &currentDrone) {
  b2Vec2 steering(0, 0);
  b2Vec2 avgVec(0, 0);
  int32 neighbours = 0;

  for (auto &drone : drones) {
    avgVec += drone->GetLinearVelocity();
    neighbours++;
  }

  if (neighbours > 0) {
    avgVec.x /= neighbours;
    avgVec.y /= neighbours;
    avgVec.Normalize();
    avgVec *= currentDrone.getMaxSpeed();

    steering = avgVec - currentDrone.getVelocity();
    clampMagnitude(steering, currentDrone.getMaxForce());
  }
  return steering;
}

b2Vec2 FlockingBehaviour::cohere(std::vector<b2Body *> &drones,
                                 Drone &currentDrone) {
  b2Vec2 steering(0, 0);
  b2Vec2 centreOfMass(0, 0);
  int32 neighbours = 0;

  for (auto &drone : drones) {
    centreOfMass += drone->GetPosition();
    neighbours++;
  }

  if (neighbours > 0) {
    centreOfMass.x /= neighbours;
    centreOfMass.y /= neighbours;
    b2Vec2 vecToCom = centreOfMass - currentDrone.getPosition();

    vecToCom.Normalize();
    vecToCom *= currentDrone.getMaxSpeed();

    steering = vecToCom - currentDrone.getVelocity();
    clampMagnitude(steering, currentDrone.getMaxForce());
  }
  return steering;
}

b2Vec2 FlockingBehaviour::separate(std::vector<b2Body *> &drones,
                                   Drone &currentDrone) {
  b2Vec2 steering(0, 0);
  b2Vec2 avgVec(0, 0);
  int32 neighbours = 0;

  for (auto &drone : drones) {
    float distance =
        b2Distance(currentDrone.getPosition(), drone->GetPosition());
    if (distance < params.separationDistance && distance > 0) {
      b2Vec2 diff = currentDrone.getPosition() - drone->GetPosition();
      diff.Normalize();
      diff.x /= distance;
      diff.y /= distance;
      avgVec += diff;
      neighbours++;
    }
  }

  if (neighbours > 0) {
    avgVec.x /= neighbours;
    avgVec.y /= neighbours;
  }
  if (avgVec.Length() > 0) {
    avgVec.Normalize();
    avgVec *= currentDrone.getMaxSpeed();

    steering = avgVec - currentDrone.getVelocity();
    clampMagnitude(steering, currentDrone.getMaxForce());
  }

  return steering;
}