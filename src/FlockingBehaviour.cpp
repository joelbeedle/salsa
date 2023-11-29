#include "FlockingBehaviour.h"

#include "Drone.h"
#include "RayCastCallback.h"

void FlockingBehaviour::execute(std::vector<Drone *> &drones,
                                Drone *currentDrone) {
  // Using ray casting to find neighbours and obstacles
  RayCastCallback callback;
  performRayCasting(currentDrone, callback);

  // Separating drones and obstacles from callback results
  std::vector<b2Body *> neighbours = callback.detectedDrones;
  std::vector<b2Vec2> obstaclePoints = callback.obstaclePoints;

  b2Vec2 alignment = align(neighbours, currentDrone);
  b2Vec2 separation = separate(neighbours, currentDrone);
  b2Vec2 cohesion = cohere(neighbours, currentDrone);
  b2Vec2 obstacleAvoidance = avoidObstacles(obstaclePoints, currentDrone);

  b2Vec2 acceleration = (alignmentWeight * alignment) +
                        (separationWeight * separation) +
                        (cohesionWeight * cohesion) +
                        (obstacleAvoidanceWeight * obstacleAvoidance);
  b2Vec2 velocity = currentDrone->getVelocity();
  b2Vec2 position = currentDrone->getPosition();

  velocity += acceleration;
  float speed = 0.01f + velocity.Length();
  b2Vec2 dir(velocity.x / speed, velocity.y / speed);

  // Clamp speed
  if (speed > maxSpeed) {
    speed = maxSpeed;
  } else if (speed < 0) {
    speed = 0.01f;
  }
  velocity = b2Vec2(dir.x * speed, dir.y * speed);

  currentDrone->getBody()->SetLinearVelocity(velocity);
  acceleration.SetZero();  // TODO: Find out implications of acceleration not
                           // being transient
}

void FlockingBehaviour::performRayCasting(Drone *currentDrone,
                                          RayCastCallback &callback) {
  // Define the ray casting range and angle
  float rayRange = viewRange;
  float deltaAngle = 15.0f;  // dividing the circle into segments

  for (float angle = 0; angle < 360; angle += deltaAngle) {
    b2Vec2 start = currentDrone->getPosition();
    b2Vec2 end = start + rayRange * b2Vec2(cosf(angle * (b2_pi / 180.0f)),
                                           sinf(angle * (b2_pi / 180.0f)));

    currentDrone->getBody()->GetWorld()->RayCast(&callback, start, end);
  }
}

b2Vec2 FlockingBehaviour::avoidObstacles(std::vector<b2Vec2> &obstaclePoints,
                                         Drone *currentDrone) {
  b2Vec2 steering(0, 0);
  int32 count = 0;

  for (auto &point : obstaclePoints) {
    b2Vec2 diff = currentDrone->getPosition() - point;
    float distance = diff.Length();
    if (distance < viewRange && distance > 0) {
      // Make sure it's weighted by the inverse distance
      diff.Normalize();
      diff *= (1.0f / distance);
      steering += diff;
      count++;
    }
  }

  if (count > 0) {
    // steering.x /= count;
    // steering.y /= count;
    steering.Normalize();
    steering *= maxSpeed;

    b2Vec2 desiredVelocity = steering - currentDrone->getVelocity();
    clampMagnitude(desiredVelocity, maxForce);
    steering = desiredVelocity;
  }

  return steering;
}

b2Vec2 FlockingBehaviour::align(std::vector<b2Body *> &drones,
                                Drone *currentDrone) {
  b2Vec2 steering(0, 0);
  b2Vec2 avgVec(0, 0);
  int32 neighbours = 0;

  for (auto &drone : drones) {
    avgVec += drone->GetLinearVelocity();
    neighbours++;
  }

  if (neighbours > 0) {
    // avgVec.x /= neighbours;
    // avgVec.y /= neighbours;
    avgVec.Normalize();
    avgVec *= maxSpeed;

    steering = avgVec - currentDrone->getVelocity();
    clampMagnitude(steering, maxForce);
  }
  return steering;
}

b2Vec2 FlockingBehaviour::cohere(std::vector<b2Body *> &drones,
                                 Drone *currentDrone) {
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
    b2Vec2 vecToCom = centreOfMass - currentDrone->getPosition();

    vecToCom.Normalize();
    vecToCom *= maxSpeed;

    steering = vecToCom - currentDrone->getVelocity();
    clampMagnitude(steering, maxForce);
  }
  return steering;
}

b2Vec2 FlockingBehaviour::separate(std::vector<b2Body *> &drones,
                                   Drone *currentDrone) {
  b2Vec2 steering(0, 0);
  b2Vec2 avgVec(0, 0);
  int32 neighbours = 0;

  for (auto &drone : drones) {
    b2Vec2 diff = currentDrone->getPosition() - drone->GetPosition();
    float distance = diff.Length();
    if (distance < separationDistance) {
      diff.Normalize();
      diff *= (1.0f / distance);
      avgVec += diff;
      neighbours++;
    }
  }

  if (neighbours > 0) {
    // avgVec.x /= neighbours;
    // avgVec.y /= neighbours;

    avgVec.Normalize();
    avgVec *= maxSpeed;

    steering = avgVec - currentDrone->getVelocity();
    clampMagnitude(steering, maxForce);
  }

  return steering;
}

void FlockingBehaviour::clampMagnitude(b2Vec2 &vector, float maxMagnitude) {
  float length = vector.Length();
  if (length > maxMagnitude && length > 0.0f) {
    vector.x /= length;
    vector.y /= length;
    vector *= maxMagnitude;
  }
}
