#include <salsa/salsa.h>

#include <iostream>
#include <valarray>
namespace salsa {

class DroneQueryCallback final : public b2QueryCallback {
 public:
  std::vector<b2Body *> foundWalls;

  bool ReportFixture(b2Fixture *fixture) override {
    if (b2Body *body = fixture->GetBody(); body->GetType() == b2_staticBody) {
      foundWalls.push_back(body);
    }
    return true;
  }
};

class FlockingBehaviour final : public Behaviour {
 private:
  std::vector<b2Body *> obstacles;

  behaviour::Parameter separation_distance_;
  behaviour::Parameter alignment_weight_;
  behaviour::Parameter cohesion_weight_;
  behaviour::Parameter separation_weight_;
  behaviour::Parameter obstacle_avoidance_weight_;

 public:
  FlockingBehaviour(const float separationDistance, const float alignmentWeight,
                    const float cohesionWeight, const float separationWeight,
                    const float obstacleAvoidanceWeight)
      : separation_distance_(separationDistance, 0.0f, 1000.0f),
        alignment_weight_(alignmentWeight, 0.0f, 2.0f),
        cohesion_weight_(cohesionWeight, 0.0f, 2.0f),
        separation_weight_(separationWeight, 0.0f, 5.0f),
        obstacle_avoidance_weight_(obstacleAvoidanceWeight, 0.0f, 5.0f) {
    // Register parameters in the map
    parameters_["Separation Distance"] = &separation_distance_;
    parameters_["Alignment Weight"] = &alignment_weight_;
    parameters_["Cohesion Weight"] = &cohesion_weight_;
    parameters_["Separation Weight"] = &separation_weight_;
    parameters_["Obstacle Avoidance Weight"] = &obstacle_avoidance_weight_;
  }

  void execute(const std::vector<std::unique_ptr<Drone>> &drones,
               Drone &currentDrone) override {
    // Using ray casting to find neighbours and obstacles
    // DroneQueryCallback queryCallback;
    // b2AABB aabb = currentDrone.getViewSensor()->GetAABB(0);
    // currentDrone.body()->GetWorld()->QueryAABB(&queryCallback, aabb);
    RayCastCallback callback;
    performRayCasting(currentDrone, callback);
    std::vector<b2Vec2> obstaclePoints = callback.obstaclePoints;
    b2Vec2 alignSteering(0, 0);
    b2Vec2 cohereSteering(0, 0);
    b2Vec2 separateSteering(0, 0);
    b2Vec2 alignAvgVec(0, 0);
    b2Vec2 separateAvgVec(0, 0);
    int32 neighbours = 0;
    b2Vec2 centreOfMass(0, 0);

    const float currentMaxSpeed = currentDrone.max_speed();
    const float currentMaxForce = currentDrone.max_force();

    for (auto &drone : drones) {
      const b2Body *body = drone->body();
      b2Vec2 bodyPos = body->GetPosition();
      if (body == currentDrone.body()) {
        continue;
      }
      const float distance = b2Distance(currentDrone.position(), bodyPos);
      if (distance > currentDrone.drone_detection_range()) {
        continue;
      }
      alignAvgVec += body->GetLinearVelocity();
      centreOfMass += bodyPos;
      if (distance < separation_distance_ && distance > 0) {
        b2Vec2 diff = currentDrone.position() - bodyPos;
        diff.Normalize();
        diff.x /= distance;
        diff.y /= distance;

        separateAvgVec += diff;
      }
      neighbours++;
    }

    if (neighbours > 0) {
      alignAvgVec.x /= neighbours;
      alignAvgVec.y /= neighbours;
      alignAvgVec.Normalize();
      alignAvgVec *= currentMaxSpeed;

      alignSteering = alignAvgVec - currentDrone.velocity();
      clampMagnitude(alignSteering, currentMaxForce);

      centreOfMass.x /= neighbours;
      centreOfMass.y /= neighbours;
      b2Vec2 vecToCom = centreOfMass - currentDrone.position();

      vecToCom.Normalize();
      vecToCom *= currentMaxSpeed;

      cohereSteering = vecToCom - currentDrone.velocity();
      clampMagnitude(cohereSteering, currentMaxForce);
      separateAvgVec.x /= neighbours;
      separateAvgVec.y /= neighbours;
    }
    if (separateAvgVec.Length() > 0) {
      separateAvgVec.Normalize();
      separateAvgVec *= currentMaxSpeed;

      separateSteering = separateAvgVec - currentDrone.velocity();
      clampMagnitude(separateSteering, currentMaxForce);
    }

    const b2Vec2 alignment = alignSteering;
    const b2Vec2 separation = separateSteering;
    const b2Vec2 cohesion = cohereSteering;
    const b2Vec2 obstacleAvoidance = avoidObstacles(obstaclePoints, currentDrone);
    const b2Vec2 toPoint = steerTo(b2Vec2(0, 0), currentDrone);
    b2Vec2 acceleration =
        (alignment_weight_ * alignment) + (separation_weight_ * separation) +
        (cohesion_weight_ * cohesion) +
        (obstacle_avoidance_weight_ * obstacleAvoidance) + 0.0 * toPoint;
    b2Vec2 velocity = currentDrone.velocity();
    b2Vec2 position = currentDrone.position();

    velocity += acceleration;
    float speed = 0.001f + velocity.Length();
    const b2Vec2 dir(velocity.x / speed, velocity.y / speed);

    // Clamp speed
    if (speed > currentMaxSpeed) {
      speed = currentMaxSpeed;
    } else if (speed < 0) {
      speed = 0.001f;
    }
    velocity = b2Vec2(dir.x * speed, dir.y * speed);

    currentDrone.body()->SetLinearVelocity(velocity);
    acceleration.SetZero();
  }

 private:
  b2Vec2 align(const std::vector<std::unique_ptr<Drone>> &drones,
               const Drone &currentDrone) {
    b2Vec2 steering(0, 0);
    b2Vec2 avgVec(0, 0);
    int32 neighbours = 0;

    for (auto &drone : drones) {
      avgVec += drone->body()->GetLinearVelocity();
      neighbours++;
    }

    if (neighbours > 0) {
      avgVec.x /= neighbours;
      avgVec.y /= neighbours;
      avgVec.Normalize();
      avgVec *= currentDrone.max_speed();

      steering = avgVec - currentDrone.velocity();
      clampMagnitude(steering, currentDrone.max_force());
    }
    return steering;
  }
  b2Vec2 separate(const std::vector<std::unique_ptr<Drone>> &drones,
                  const Drone &currentDrone) {
    b2Vec2 steering(0, 0);
    b2Vec2 avgVec(0, 0);
    int32 neighbours = 0;

    for (auto &drone : drones) {
      float distance =
          b2Distance(currentDrone.position(), drone->body()->GetPosition());
      if (distance < separation_distance_ && distance > 0) {
        b2Vec2 diff = currentDrone.position() - drone->body()->GetPosition();
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
      avgVec *= currentDrone.max_speed();

      steering = avgVec - currentDrone.velocity();
      clampMagnitude(steering, currentDrone.max_force());
    }

    return steering;
  }
  b2Vec2 cohere(const std::vector<std::unique_ptr<Drone>> &drones,
                const Drone &currentDrone) {
    b2Vec2 steering(0, 0);
    b2Vec2 centreOfMass(0, 0);
    int32 neighbours = 0;

    for (auto &drone : drones) {
      centreOfMass += drone->body()->GetPosition();
      neighbours++;
    }

    if (neighbours > 0) {
      centreOfMass.x /= neighbours;
      centreOfMass.y /= neighbours;
      b2Vec2 vecToCom = centreOfMass - currentDrone.position();

      vecToCom.Normalize();
      vecToCom *= currentDrone.max_speed();

      steering = vecToCom - currentDrone.velocity();
      clampMagnitude(steering, currentDrone.max_force());
    }
    return steering;
  }
};

auto flocking = behaviour::Registry::get().add(
    "Flocking",
    std::make_unique<salsa::FlockingBehaviour>(250.0, 1.6, 1.0, 3.0, 4.0));
}  // namespace salsa