#include "UniformRandomWalkBehaviour.h"

#include <iostream>

#include "Drone.h"
#include "box2d/box2d.h"

UniformRandomWalkBehaviour::UniformRandomWalkBehaviour(
    const UniformRandomWalkParameters &params)
    : params(params) {
  std::srand(std::time(nullptr));  // Initialize random seed
}

void UniformRandomWalkBehaviour::performRayCasting(Drone *currentDrone,
                                                   RayCastCallback &callback) {
  // Define the ray casting range and angle
  float rayRange = currentDrone->getViewRange();
  ;
  float deltaAngle = 15.0f;  // dividing the circle into segments

  for (float angle = 0; angle < 360; angle += deltaAngle) {
    b2Vec2 start = currentDrone->getPosition();
    b2Vec2 end = start + rayRange * b2Vec2(cosf(angle * (b2_pi / 180.0f)),
                                           sinf(angle * (b2_pi / 180.0f)));

    currentDrone->getBody()->GetWorld()->RayCast(&callback, start, end);
  }
}

void UniformRandomWalkBehaviour::execute(std::vector<Drone *> &drones,
                                         Drone *currentDrone) {
  // Ensure there is a timer info for the current drone
  if (droneTimers.find(currentDrone) == droneTimers.end()) {
    droneTimers[currentDrone] = DroneTimerInfo();
    droneTimers[currentDrone].desiredVelocity = currentDrone->getVelocity();
  }
  RayCastCallback callback;
  performRayCasting(currentDrone, callback);

  std::vector<b2Vec2> obstaclePoints = callback.obstaclePoints;
  std::vector<b2Body *> neighbours = callback.detectedDrones;

  DroneTimerInfo &timerInfo = droneTimers[currentDrone];
  timerInfo.elapsedTimeSinceLastForce += params.deltaTime;
  b2Vec2 force = b2Vec2(0, 0);
  b2Vec2 steer = b2Vec2(0, 0);
  b2Vec2 obstacleAvoidance = avoidObstacles(obstaclePoints, currentDrone);
  b2Vec2 neighbourAvoidance = avoidDrones(neighbours, currentDrone);

  // Check if it's time to apply a new random force
  if (timerInfo.elapsedTimeSinceLastForce >= timerInfo.randomTimeInterval) {
    float angle = static_cast<float>(std::rand()) / RAND_MAX * 2 * M_PI;

    // New desired velocity based on random angle
    timerInfo.desiredVelocity =
        b2Vec2(std::cos(angle) * currentDrone->getMaxSpeed(),
               std::sin(angle) * currentDrone->getMaxSpeed());

    // Reset the timer and generate a new random time interval for this drone
    timerInfo.elapsedTimeSinceLastForce = 0.0f;
    timerInfo.randomTimeInterval = generateRandomTimeInterval();
  }

  steer = timerInfo.desiredVelocity - currentDrone->getVelocity();
  clampMagnitude(steer, currentDrone->getMaxForce());

  b2Vec2 velocity = currentDrone->getVelocity();
  b2Vec2 position = currentDrone->getPosition();
  b2Vec2 acceleration = (params.forceWeight * steer) +
                        (params.obstacleAvoidanceWeight * obstacleAvoidance) +
                        neighbourAvoidance;

  velocity += acceleration;
  float speed = 0.001f + velocity.Length();
  b2Vec2 dir(velocity.x / speed, velocity.y / speed);

  // Clamp speed
  if (speed > currentDrone->getMaxSpeed()) {
    speed = currentDrone->getMaxSpeed();
  } else if (speed < 0) {
    speed = 0.001f;
  }
  velocity = b2Vec2(dir.x * speed, dir.y * speed);

  currentDrone->getBody()->SetLinearVelocity(velocity);
}