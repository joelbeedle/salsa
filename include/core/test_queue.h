/// \file test_stack.h
/// Contains the static `TestStack` class, which is used to manage the running
/// of simulation tests.
#ifndef SWARM_SIM_CORE_TEST_STACK_H
#define SWARM_SIM_CORE_TEST_STACK_H

#include <box2d/box2d.h>

#include <queue>
#include <stdexcept>
#include <variant>

#include "behaviours/behaviour.h"
#include "behaviours/parameter.h"
#include "core/map.h"
#include "entity/drone_configuration.h"
#include "utils/base_contact_listener.h"
namespace swarm {

struct TestConfig {
  typedef std::unordered_map<std::string, behaviour::Parameter *> Parameters;
  typedef std::unordered_map<std::string, float> FloatParameters;

  std::string behaviour_name;
  std::variant<Parameters, FloatParameters> parameters;
  DroneConfiguration *drone_config;
  map::Map map;
  int num_drones;
  int num_targets;
  float time_limit;
  std::string target_type = "null";
  std::any target_parameters;
  BaseContactListener *contact_listener;
  bool keep = true;
  // FUTURE: std::function<void()> drone_setup;
  // FUTURE: std::function<void()> target_setup;
};

class TestQueue {
 private:
  static std::vector<TestConfig> tests_;

 public:
  static void push(TestConfig test);
  static TestConfig pop();
  static TestConfig peek();
  static int size() { return tests_.size(); }
  static std::vector<TestConfig> &getTests() { return tests_; }
  static bool isEmpty() { return tests_.empty(); }
};

}  // namespace swarm

#endif  // SWARM_SIM_CORE_TEST_STACK_H