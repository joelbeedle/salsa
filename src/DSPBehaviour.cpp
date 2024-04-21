#include "DSPBehaviour.h"

#include "Drone.h"
#include "RayCastCallback.h"
#include "box2d/box2d.h"

void DSPBehaviour::execute(const std::vector<std::unique_ptr<Drone>> &drones,
                           Drone &currentDrone) {
  if (droneInformation.find(&currentDrone) == droneInformation.end()) {
    droneInformation[&currentDrone] = DroneInfo();
    DSPPoint *dsp = new DSPPoint(currentDrone.getBody()->GetWorld(),
                                 currentDrone.getPosition());
    droneInformation[&currentDrone].dsp = dsp;
    dspPoints.push_back(dsp);
  }
  RayCastCallback callback;
  performRayCasting(currentDrone, callback);
  std::vector<b2Body *> bodies;
  // collect DSP points from other drones
  DroneInfo &droneInfo = droneInformation[&currentDrone];

  for (auto &drone : drones) {
    bodies.push_back(drone.get()->getBody());
  }
  std::vector<b2Body *> neighbours = bodies;
  std::vector<b2Vec2> obstaclePoints = callback.obstaclePoints;
  b2Vec2 obstacleAvoidance = avoidObstacles(obstaclePoints, currentDrone);
  b2Vec2 neighbourAvoidance = avoidDrones(neighbours, currentDrone);

  b2Vec2 bspPos = droneInfo.dsp->body->GetPosition();
  b2Vec2 force(0.0f, 0.0f);
  float forceMag = 0.0f;
  b2Vec2 overallDir(0.0f, 0.0f);
  for (auto &point : dspPoints) {
    if (point != droneInfo.dsp) {
      b2Vec2 pointPos = point->body->GetPosition();
      float forceMagnitude = droneInfo.dsp->gravDSPForce(bspPos, pointPos);
      b2Vec2 direction = directionTo(bspPos, pointPos);
      direction.Normalize();
      force += forceMagnitude *
               direction;  // Calculate and accumulate the correct force vector
    }
  }

  // Clamp the force if it exceeds the maximum allowable force
  if (force.Length() > 4000.0f) {
    force.Normalize();
    force *= 4000.0f;
  }

  // Update DSP position for this drone
  droneInfo.dsp->body->SetLinearVelocity(force);

  b2Vec2 velocity = currentDrone.getVelocity();
  b2Vec2 position = currentDrone.getPosition();
  b2Vec2 acceleration(0, 0);
  b2Vec2 steer(0, 0);

  float distanceToDSP = b2Distance(position, bspPos);
  if (distanceToDSP < 5.0f) {
    if (!droneInfo.beginWalk && droneInfo.elapsedTime == 0.0f) {
      // Start the random walk if not already started and timer is reset
      droneInfo.beginWalk = true;
      droneInfo.elapsedTime = 0.0f;
    }
  }
  // Handle random walk logic
  if (droneInfo.beginWalk) {
    if (droneInfo.elapsedTime >= droneInfo.timeToWalk) {
      // Walk time is up, stop walking
      droneInfo.beginWalk = false;
      droneInfo.elapsedTime = 0.0f;
    } else {
      // Continue walking
      if (droneInfo.elapsedTimeSinceLastForce >= droneInfo.randomTimeInterval) {
        // Change direction at regular intervals
        float angle = static_cast<float>(std::rand()) / RAND_MAX * 2 * M_PI;
        droneInfo.desiredVelocity =
            b2Vec2(std::cos(angle) * currentDrone.getMaxSpeed(),
                   std::sin(angle) * currentDrone.getMaxSpeed());

        droneInfo.elapsedTimeSinceLastForce =
            0.0f;  // Reset the timer for force update
        droneInfo.randomTimeInterval =
            generateRandomTimeInterval();  // Next interval
      }

      droneInfo.elapsedTimeSinceLastForce += (1.0 / 30.0f);
      droneInfo.elapsedTime += (1.0 / 30.0f);

      // Apply random walk velocity as steering force
      b2Vec2 steer = droneInfo.desiredVelocity - currentDrone.getVelocity();
      clampMagnitude(steer, currentDrone.getMaxForce());
      acceleration += steer;
    }
  } else {
    // If not walking, drone should head towards the DSP point
    b2Vec2 dir = directionTo(position, bspPos);
    dir.Normalize();
    b2Vec2 steering = currentDrone.getMaxSpeed() * dir;
    clampMagnitude(steering, currentDrone.getMaxForce());
    acceleration += steering;
  }
  acceleration += neighbourAvoidance + (3.0f * obstacleAvoidance);
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
  acceleration.SetZero();  // TODO: Find out implications of acceleration not
                           // being transient
}

void DSPBehaviour::clean(const std::vector<std::unique_ptr<Drone>> &drones) {
  for (auto &point : dspPoints) {
    b2World *world = point->body->GetWorld();
    world->DestroyBody(point->body);
  }
  dspPoints.clear();
  droneInformation.clear();
}