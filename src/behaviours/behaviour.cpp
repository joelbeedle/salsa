#include "salsa/behaviours/behaviour.h"

#include "box2d/box2d.h"
#include "salsa/entity/drone.h"

using namespace salsa;

void Behaviour::setParameters(
    const std::unordered_map<std::string, float> &parameters) {
  std::unordered_map<std::string, behaviour::Parameter *> parameters_ =
      getParameters();
  for (auto &[name, val] : parameters) {
    if (parameters_.find(name) != parameters_.end()) {
      parameters_[name]->value() = val;
    }
  }
}

void Behaviour::setParameters(
    const std::unordered_map<std::string, salsa::behaviour::Parameter *>
        &parameters) {
  auto parameters_ = getParameters();  // Update the existing parameters
  for (const auto &[name, val] : parameters) {
    if (parameters_.find(name) != parameters_.end()) {
      parameters_[name] = val;
    }
  }
}

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
  b2Body *currentBody = currentDrone.body();

  for (auto &drone : neighbours) {
    if (drone != currentBody) {
      float d = b2Distance(currentDrone.position(), drone->GetPosition());
      if (d < currentDrone.camera_view_range() && d > 0) {
        b2Vec2 diff = currentDrone.position() - drone->GetPosition();
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
    steering *= currentDrone.max_speed();

    steering -= currentDrone.velocity();
    clampMagnitude(steering, currentDrone.max_force());
  }

  return steering;
}

b2Vec2 Behaviour::avoidObstacles(std::vector<b2Vec2> &obstaclePoints,
                                 Drone &currentDrone) {
  b2Vec2 steering(0, 0);
  int32 count = 0;

  for (auto &point : obstaclePoints) {
    float distance = b2Distance(currentDrone.position(), point);
    if (distance < currentDrone.obstacle_view_range() && distance > 0) {
      b2Vec2 diff = currentDrone.position() - point;
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
    steering *= currentDrone.max_speed();

    steering -= currentDrone.velocity();
    clampMagnitude(steering, currentDrone.max_force());
  }

  return steering;
}

void Behaviour::performRayCasting(Drone &currentDrone,
                                  RayCastCallback &callback) {
  float rayRange = currentDrone.obstacle_view_range();
  float deltaAngle = 45.0f;

  for (float angle = 0; angle < 360; angle += deltaAngle) {
    b2Vec2 start = currentDrone.position();
    b2Vec2 end = start + rayRange * b2Vec2(cosf(angle * (b2_pi / 180.0f)),
                                           sinf(angle * (b2_pi / 180.0f)));

    currentDrone.body()->GetWorld()->RayCast(&callback, start, end);
  }
}

b2Vec2 Behaviour::steerTo(b2Vec2 target, Drone &currentDrone) {
  b2Vec2 position = currentDrone.position();
  b2Vec2 desired = target - position;
  float d = desired.Length();
  b2Vec2 steer(0.0f, 0.0f);
  desired.Normalize();
  desired.x *= currentDrone.max_speed();
  desired.y *= currentDrone.max_speed();
  steer = desired - currentDrone.velocity();
  clampMagnitude(steer, currentDrone.max_force());
  return steer;
}