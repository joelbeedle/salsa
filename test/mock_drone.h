#include "drones/drone.h"
#include "gmock/gmock.h"
class MockDrone : public swarm::Drone {
 public:
  MOCK_METHOD(void, update, (const std::vector<std::shared_ptr<Drone>>&), ());
  MOCK_METHOD(void, setMaxForce, (float), ());
  MOCK_METHOD(void, setMaxSpeed, (float), ());
  MOCK_METHOD(void, setViewRange, (float), ());
  MOCK_METHOD(void, setObstacleViewRange, (float), ());
  MOCK_METHOD(void, updateSensorRange, (), ());
};
