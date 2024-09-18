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

std::unordered_map<std::string, float> Behaviour::getParameterValues() {
  std::unordered_map<std::string, float> values;
  for (const auto &[name, param] : parameters_) {
    values[name] = param->value();
  }
  return values;
}

std::unordered_map<std::string, float> Behaviour::convertParametersToFloat(
    const std::unordered_map<std::string, behaviour::Parameter *>& parameters) {
  std::unordered_map<std::string, float> values;
  for (const auto &[name, param] : parameters) {
    values[name] = param->value();
  }
  return values;
}

std::unordered_map<std::string, float> Behaviour::convertParametersToFloat(
    std::unordered_map<std::string, float> parameters) {
  return parameters;
}

// namespace behaviours {
void Behaviour::clampMagnitude(b2Vec2 &vector, const float maxMagnitude) {
  if (float lengthSquared = vector.LengthSquared(); lengthSquared > maxMagnitude * maxMagnitude && lengthSquared > 0) {
    vector.Normalize();
    vector *= maxMagnitude;
  }
}

b2Vec2 Behaviour::avoidDrones(const std::vector<b2Body *> &neighbours,
                              const Drone &currentDrone) {
  b2Vec2 steering(0, 0);
  int32 count = 0;
  const b2Body *currentBody = currentDrone.body();

  for (auto &drone : neighbours) {
    if (drone != currentBody) {
      if (const float d = b2Distance(currentDrone.position(), drone->GetPosition()); d < currentDrone.camera_view_range() && d > 0) {
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

b2Vec2 Behaviour::avoidObstacles(const std::vector<b2Vec2> &obstaclePoints,
                                 const Drone &currentDrone) {
  b2Vec2 steering(0, 0);
  int32 count = 0;

  for (auto &point : obstaclePoints) {
    if (float distance = b2Distance(currentDrone.position(), point); distance < currentDrone.obstacle_view_range() && distance > 0) {
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

void Behaviour::performRayCasting(const Drone &currentDrone,
                                  RayCastCallback &callback) {
  const float rayRange = currentDrone.obstacle_view_range();
   constexpr float deltaAngle = 45.0f;

  for (float angle = 0; angle < 360; angle += deltaAngle) {
    b2Vec2 start = currentDrone.position();
    b2Vec2 end = start + rayRange * b2Vec2(cosf(angle * (b2_pi / 180.0f)),
                                           sinf(angle * (b2_pi / 180.0f)));

    currentDrone.body()->GetWorld()->RayCast(&callback, start, end);
  }
}

b2Vec2 Behaviour::steerTo(const b2Vec2 target, const Drone &currentDrone) {
  const b2Vec2 position = currentDrone.position();
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