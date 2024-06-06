// DroneConfiguration.h
#ifndef SWARM_SIM_UTILS_DRONE_CONFIGURATION_H
#define SWARM_SIM_UTILS_DRONE_CONFIGURATION_H

namespace swarm_sim {
class DroneConfiguration {
 public:
  float cameraViewRange;
  float obstacleViewRange;
  float maxSpeed;
  float maxForce;
  float radius;
  float mass;
  float droneDetectionRange;

  DroneConfiguration(float cameraView, float obstacleView, float maxSpd,
                     float maxFrc, float rad, float mss, float droneDetectRange)
      : cameraViewRange(cameraView),
        obstacleViewRange(obstacleView),
        maxSpeed(maxSpd),
        maxForce(maxFrc),
        radius(rad),
        mass(mss),
        droneDetectionRange(droneDetectRange) {}
};

}  // namespace swarm_sim

#endif  // SWARM_SIM_UTILS_DRONE_CONFIGURATION_H