#ifndef SWARM_TESTS_MOCK_BEHAVIOUR_H
#define SWARM_TESTS_MOCK_BEHAVIOUR_H

#include "gmock/gmock.h"
#include "salsa/behaviours/behaviour.h"

class MockBehaviour : public swarm::Behaviour {
 public:
  MOCK_METHOD(void, execute,
              (const std::vector<std::unique_ptr<swarm::Drone>> &drones,
               swarm::Drone &currentDrone),
              (override));
  MOCK_METHOD(void, clean,
              (const std::vector<std::unique_ptr<swarm::Drone>> &drones),
              (override));
};

#endif  // SWARM_TESTS_MOCK_BEHAVIOUR_H