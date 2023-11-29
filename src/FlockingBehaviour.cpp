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

  b2Vec2 acceleration = alignment + separation + cohesion + obstacleAvoidance;
  b2Vec2 velocity = currentDrone->getVelocity();
  b2Vec2 position = currentDrone->getPosition();

  float velocityMagnitude = velocity.Length();
  if (velocityMagnitude > 5.0f) {  // TODO: replace 5.0f with maxSpeed
    if (velocityMagnitude > 0.0f) {
      velocity *= (1.0f / velocityMagnitude);  // Normalize by multiplying with
                                               // the inverse of the magnitude
    }

    velocity *= 5.0f;
  }
  currentDrone->getBody()->SetLinearVelocity(velocity + acceleration);
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
    if (distance < 100.0f && distance > 0) {
      diff *= (1.0f / distance);  // Normalize and weight inversely by distance
      steering += diff;
      count++;
    }
  }

  if (count > 0) {
    steering.Normalize();
    steering *= maxSpeed;
    b2Vec2 desiredVelocity = steering - currentDrone->getVelocity();
    if (desiredVelocity.Length() > maxForce) {
      desiredVelocity.Normalize();
      desiredVelocity *= maxForce;
    }
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
    avgVec.x /= neighbours;
    avgVec.y /= neighbours;
    float length = avgVec.Length();
    if (length > 0) {
      avgVec *= (1.0f / length);
      avgVec *= maxSpeed;
    }
    steering = avgVec - currentDrone->getVelocity();
  }
  return avgVec;
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
    float length = vecToCom.Length();
    if (length > 0) {
      vecToCom *= (1.0f / length);
      vecToCom *= maxSpeed;
    }
    steering = vecToCom - currentDrone->getVelocity();
    length = steering.Length();
    if (length > maxForce) {
      steering *= (1.0f / length);
      steering *= maxForce;
    }
  }
  return steering;
}

b2Vec2 FlockingBehaviour::separate(std::vector<b2Body *> &drones,
                                   Drone *currentDrone) {
  b2Vec2 steering(0, 0);
  b2Vec2 avgVec(0, 0);
  int32 neighbours = 0;

  for (auto &drone : drones) {
    float distance =
        (drone->GetPosition() - currentDrone->getPosition()).LengthSquared();
    if (distance < separationDistance * separationDistance) {
      b2Vec2 diff = currentDrone->getPosition() - drone->GetPosition();
      diff.Normalize();
      diff.x /= distance;  // Weight by distance
      diff.y /= distance;  // Weight by distance
      avgVec += diff;
      neighbours++;
    }
  }

  if (neighbours > 0) {
    avgVec.x /= neighbours;
    avgVec.y /= neighbours;
    float length = avgVec.Length();
    if (length > 0) {
      avgVec *= (1.0f / length);
      avgVec *= maxSpeed;
      steering = avgVec - currentDrone->getVelocity();
      length = steering.Length();
      if (length > maxForce) {
        steering *= (1.0f / length);
        steering *= maxForce;
      }
    }
  }

  return steering;
}