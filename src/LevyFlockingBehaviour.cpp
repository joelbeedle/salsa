#include "LevyFlockingBehaviour.h"

#include <iostream>
#include <random>
#include <valarray>

#include "Drone.h"
#include "RayCastCallback.h"
#include "box2d/box2d.h"

void LevyFlockingBehaviour::execute(
    const std::vector<std::unique_ptr<Drone>> &drones, Drone &currentDrone) {
  // Using ray casting to find neighbours and obstacles
  if (droneInformation.find(&currentDrone) == droneInformation.end()) {
    droneInformation[&currentDrone] = DroneInfo();
  }

  RayCastCallback callback;
  performRayCasting(currentDrone, callback);
  std::vector<b2Body *> bodies;
  for (auto &drone : drones) {
    bodies.push_back(drone.get()->getBody());
  }

  // Separating drones and obstacles from callback results
  std::vector<b2Body *> neighbours = bodies;
  std::vector<b2Vec2> obstaclePoints = callback.obstaclePoints;

  DroneInfo &droneInfo = droneInformation[&currentDrone];
  b2Vec2 velocity = currentDrone.getVelocity();
  b2Vec2 position = currentDrone.getPosition();
  b2Vec2 acceleration(0, 0);
  b2Vec2 levyVector(0, 0);

  b2Vec2 alignment = align(neighbours, currentDrone);
  b2Vec2 separation = separate(neighbours, currentDrone);
  b2Vec2 cohesion = cohere(neighbours, currentDrone);
  b2Vec2 obstacleAvoidance = avoidObstacles(obstaclePoints, currentDrone);
  b2Vec2 droneAvoidance = avoidDrones(bodies, currentDrone);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, 1);

  if (droneInfo.isExecuting) {
    // we are currently in a levy step
    droneInfo.accumulatedDistance += velocity.Length() * (1.0f / 30.0f);

    b2Vec2 dir = droneInfo.levyDirection;
    dir.Normalize();
    dir *= currentDrone.getMaxSpeed();
    b2Vec2 steering = dir - currentDrone.getVelocity();
    clampMagnitude(steering, currentDrone.getMaxForce());
    acceleration =
        (params.levyWeight * steering) + (params.cohesionWeight * cohesion) +
        (params.obstacleAvoidanceWeight * obstacleAvoidance) + droneAvoidance;

    if (droneInfo.accumulatedDistance >= droneInfo.stepLength) {
      // finish walk
      droneInfo.accumulatedDistance = 0.0f;
      droneInfo.isExecuting = false;
    }

  } else if (dis(gen) < 0.02) {
    // we are not in a levy step, and we should be
    droneInfo.levyPoint = levy(2.8f);
    droneInfo.stepLength =
        sqrt(pow(droneInfo.levyPoint.x, 2) + pow(droneInfo.levyPoint.y, 2));
    droneInfo.accumulatedDistance = 0.0f;
    droneInfo.levyDirection = generateRandomDirection();
    droneInfo.isExecuting = true;

    b2Vec2 dir = droneInfo.levyDirection;
    dir.Normalize();
    dir *= currentDrone.getMaxSpeed();

    b2Vec2 steering = dir - currentDrone.getVelocity();
    clampMagnitude(steering, currentDrone.getMaxForce());
    acceleration =
        (params.levyWeight * steering) + (params.cohesionWeight * cohesion) +
        (params.obstacleAvoidanceWeight * obstacleAvoidance) + droneAvoidance;

  } else {
    // usual flocking behaviour
    acceleration = (params.alignmentWeight * alignment) +
                   (params.separationWeight * separation) +
                   (params.cohesionWeight * cohesion) +
                   (params.obstacleAvoidanceWeight * obstacleAvoidance);
  }
  velocity += acceleration;

  // Update drone's velocity and position
  float speed = 0.001f + velocity.Length();
  b2Vec2 dir(velocity.x / speed, velocity.y / speed);
  clampMagnitude(velocity, currentDrone.getMaxSpeed());

  currentDrone.getBody()->SetLinearVelocity(velocity);
}

b2Vec2 LevyFlockingBehaviour::align(std::vector<b2Body *> &drones,
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

b2Vec2 LevyFlockingBehaviour::cohere(std::vector<b2Body *> &drones,
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

b2Vec2 LevyFlockingBehaviour::separate(std::vector<b2Body *> &drones,
                                       Drone &currentDrone) {
  b2Vec2 steering(0, 0);
  b2Vec2 avgVec(0, 0);
  int32 neighbours = 0;

  for (auto &drone : drones) {
    float distance =
        (currentDrone.getPosition() - drone->GetPosition()).Length();
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