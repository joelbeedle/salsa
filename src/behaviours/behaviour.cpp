#include "behaviours/behaviour.h"

#include "box2d/box2d.h"
#include "entity/drone.h"

namespace swarm {
// namespace behaviours {
void Behaviour::clampMagnitude(b2Vec2 &vector, const float maxMagnitude) {
  float lengthSquared = vector.LengthSquared();
  if (lengthSquared > maxMagnitude * maxMagnitude && lengthSquared > 0) {
    vector.Normalize();
    vector *= maxMagnitude;
  }
}

b2Vec2 Behaviour::avoidDrones(std::vector<b2Body *> &neighbours,
                              Drone &currentDrone) {
  b2Vec2 steering(0, 0);
  int32 count = 0;
  b2Body *currentBody = currentDrone.getBody();

  for (auto &drone : neighbours) {
    if (drone != currentBody) {
      float d = b2Distance(currentDrone.getPosition(), drone->GetPosition());
      if (d < currentDrone.getViewRange() && d > 0) {
        b2Vec2 diff = currentDrone.getPosition() - drone->GetPosition();
        diff.Normalize();
        diff.x /= d;
        diff.y /= d;
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

b2Vec2 Behaviour::avoidObstacles(std::vector<b2Vec2> &obstaclePoints,
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

void Behaviour::performRayCasting(Drone &currentDrone,
                                  RayCastCallback &callback) {
  float rayRange = currentDrone.getObstacleViewRange();
  float deltaAngle = 45.0f;

  for (float angle = 0; angle < 360; angle += deltaAngle) {
    b2Vec2 start = currentDrone.getPosition();
    b2Vec2 end = start + rayRange * b2Vec2(cosf(angle * (b2_pi / 180.0f)),
                                           sinf(angle * (b2_pi / 180.0f)));

    currentDrone.getBody()->GetWorld()->RayCast(&callback, start, end);
  }
}

b2Vec2 Behaviour::steerTo(b2Vec2 target, Drone &currentDrone) {
  b2Vec2 position = currentDrone.getPosition();
  b2Vec2 desired = target - position;
  float d = desired.Length();
  b2Vec2 steer(0.0f, 0.0f);
  desired.Normalize();
  desired.x *= currentDrone.getMaxSpeed();
  desired.y *= currentDrone.getMaxSpeed();
  steer = desired - currentDrone.getVelocity();
  clampMagnitude(steer, currentDrone.getMaxForce());
  return steer;
}
// }  // namespace behaviours
}  // namespace swarm