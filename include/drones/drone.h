// Drone.h
#ifndef SWARM_SIM_DRONES_DRONE_H
#define SWARM_SIM_DRONES_DRONE_H

#include <box2d/box2d.h>

// #include <SFML/Graphics.hpp>

#include <memory>

#include "behaviours/behaviour.h"
#include "tree.h"
#include "utils/drone_configuration.h"
namespace swarm {
class Drone {
 private:
  std::vector<Tree *> foundDiseasedTrees;
  std::vector<Tree *> foundTrees;
  std::vector<b2Vec2 *> foundDiseasedTreePositions;
  b2Body *body;
  b2Fixture *viewSensor;  // The view range sensor
  Behaviour *behaviour;
  float cameraViewRange;
  float obstacleViewRange;
  float droneDetectionRange;
  float maxSpeed;
  float maxForce;
  float radius;
  float mass;

 public:
  Drone(b2World *world, const b2Vec2 &position, Behaviour &behaviour,
        const DroneConfiguration &config);
  ~Drone();

  void update(const std::vector<std::unique_ptr<Drone>> &drones);
  void updateSensorRange();

  void foundDiseasedTree(Tree *tree);
  void foundTree(Tree *tree);

  // Accessors and Mutators
  void setBehaviour(Behaviour &newBehaviour) { behaviour = &newBehaviour; }

  b2Body *getBody() { return body; }
  b2Vec2 getVelocity() { return body->GetLinearVelocity(); }
  b2Vec2 getPosition() { return body->GetPosition(); }

  float getViewRange() { return cameraViewRange; }
  void setViewRange(float newRange) { cameraViewRange = newRange; }

  float getDroneDetectionRange() { return droneDetectionRange; }
  void setDroneDetectionRange(float newRange) {
    droneDetectionRange = newRange;
  }

  float getObstacleViewRange() { return obstacleViewRange; }
  void setObstacleViewRange(float newRange) { obstacleViewRange = newRange; }

  float getMaxSpeed() { return maxSpeed; }
  void setMaxSpeed(float newSpeed) { maxSpeed = newSpeed; }

  float getMaxForce() { return maxForce; }
  void setMaxForce(float newForce) { maxForce = newForce; }

  float getRadius() { return radius; }

  std::vector<Tree *> &getFoundDiseasedTrees() { return foundDiseasedTrees; }
  std::vector<b2Vec2 *> getFoundDiseasedTreePositions() {
    return foundDiseasedTreePositions;
  }

  std::vector<Tree *> getFoundTrees() { return foundTrees; }

  void clearLists();
};

}  // namespace swarm

#endif  // SWARM_SIM_DRONES_DRONE_H