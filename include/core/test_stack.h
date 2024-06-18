/// \file test_stack.h
/// Contains the static `TestStack` class, which is used to manage the running
/// of simulation tests.
#ifndef SWARM_SIM_CORE_TEST_STACK_H
#define SWARM_SIM_CORE_TEST_STACK_H

#include <box2d/box2d.h>

#include <stack>
#include <stdexcept>
#include <variant>

#include "behaviours/behaviour.h"
#include "behaviours/parameter.h"
#include "utils/drone_configuration.h"

namespace swarm {

struct TestConfig {
  typedef std::unordered_map<std::string, behaviour::Parameter *> Parameters;
  typedef std::unordered_map<std::string, float> FloatParameters;

  const std::string &behaviour_name;
  std::variant<Parameters, FloatParameters> parameters;
  DroneConfiguration *drone_config;
  b2World *world;
  int num_drones;
  int num_targets;
  float time_limit;
  // FUTURE: std::function<void()> drone_setup;
  // FUTURE: std::function<void()> target_setup;
};

class TestStack {
 private:
  static std::stack<TestConfig> tests_;

 public:
  static void push(TestConfig test);
  static TestConfig pop();
  static int size() { return tests_.size(); }
  static bool isEmpty() { return tests_.empty(); }
};

}  // namespace swarm

#endif  // SWARM_SIM_CORE_TEST_STACK_H