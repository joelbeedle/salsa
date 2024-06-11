/// \file test_stack.h
/// Contains the static `TestStack` class, which is used to manage the running
/// of simulation tests.
#ifndef SWARM_SIM_CORE_TEST_STACK_H
#define SWARM_SIM_CORE_TEST_STACK_H

#include <box2d/box2d.h>

#include <stack>

#include "behaviours/behaviour.h"
#include "behaviours/parameter.h"
#include "utils/drone_configuration.h"

namespace swarm {

struct BehaviourWrapper {
  Behaviour *behaviour;
  std::vector<behaviour::Parameter *> parameters;
};

struct Test {
  BehaviourWrapper *behaviourWrapper;
  DroneConfiguration *config;
};

class TestStack {
 private:
  static std::stack<Test> tests_;

 public:
  static void pushTest(BehaviourWrapper *wrapped_behaviour,
                       DroneConfiguration *config);
};

}  // namespace swarm

#endif  // SWARM_SIM_CORE_TEST_STACK_H