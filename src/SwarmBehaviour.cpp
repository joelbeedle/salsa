#include "SwarmBehaviour.h"

#include "Drone.h"
#include "box2d/box2d.h"

b2Vec2 SwarmBehaviour::avoidDrones(std::vector<b2Body *> &neighbours,
                                   Drone &currentDrone) {
  b2Vec2 steering(0, 0);
  int32 count = 0;
  b2Body *currentBody = currentDrone.getBody();

  for (auto &drone : neighbours) {
    if (drone != currentBody) {
      float distance =
          b2Distance(currentDrone.getPosition(), drone->GetPosition());
      if (distance < currentDrone.getViewRange() && distance > 0) {
        b2Vec2 diff = currentDrone.getPosition() - drone->GetPosition();
        diff.Normalize();
        diff.x /= distance;
        diff.y /= distance;
        steering += diff;
        count++;
      }
    }
  }

  if (count > 0) {
    steering.x /= count;
    steering.y /= count;
    steering.Normalize();
    steering *= currentDrone.getMaxSpeed();

    steering -= currentDrone.getVelocity();
    clampMagnitude(steering, currentDrone.getMaxForce());
  }

  return steering;
}

b2Vec2 SwarmBehaviour::avoidObstacles(std::vector<b2Vec2> &obstaclePoints,
                                      Drone &currentDrone) {
  b2Vec2 steering(0, 0);
  int32 count = 0;

  for (auto &point : obstaclePoints) {
    float distance = b2Distance(currentDrone.getPosition(), point);
    if (distance < currentDrone.getViewRange() && distance > 0) {
      b2Vec2 diff = currentDrone.getPosition() - point;
      // Make sure it's weighted by the inverse distance
      diff.Normalize();
      diff.x /= distance;
      diff.y /= distance;
      steering += diff;
      count++;
    }
  }

  if (count > 0) {
    steering.x /= count;
    steering.y /= count;
    steering.Normalize();
    steering *= currentDrone.getMaxSpeed();

    steering -= currentDrone.getVelocity();
    clampMagnitude(steering, currentDrone.getMaxForce());
  }

  return steering;
}

void SwarmBehaviour::performRayCasting(Drone &currentDrone,
                                       RayCastCallback &callback) {
  // Define the ray casting range and angle
  float rayRange = currentDrone.getObstacleViewRange();
  float deltaAngle = 45.0f;  // dividing the circle into segments

  for (float angle = 0; angle < 360; angle += deltaAngle) {
    b2Vec2 start = currentDrone.getPosition();
    b2Vec2 end = start + rayRange * b2Vec2(cosf(angle * (b2_pi / 180.0f)),
                                           sinf(angle * (b2_pi / 180.0f)));

    currentDrone.getBody()->GetWorld()->RayCast(&callback, start, end);
  }
}