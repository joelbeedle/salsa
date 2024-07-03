#ifndef SWARM_TESTS_MOCK_BEHAVIOUR_H
#define SWARM_TESTS_MOCK_BEHAVIOUR_H

#include "gmock/gmock.h"
#include "salsa/behaviours/behaviour.h"

class MockBehaviour : public salsa::Behaviour {
 public:
  MOCK_METHOD(void, execute,
              (const std::vector<std::unique_ptr<salsa::Drone>> &drones,
               salsa::Drone &currentDrone),
              (override));
  MOCK_METHOD(void, clean,
              (const std::vector<std::unique_ptr<salsa::Drone>> &drones),
              (override));
};

#endif  // SWARM_TESTS_MOCK_BEHAVIOUR_H