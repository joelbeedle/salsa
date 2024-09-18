/// @file test_queue.h
/// @brief Contains the static `TestQueue` class, which is used to manage the
/// running of simulation tests.
#ifndef SWARM_SIM_CORE_TEST_STACK_H
#define SWARM_SIM_CORE_TEST_STACK_H

#include <box2d/box2d.h>

#include <queue>
#include <stdexcept>
#include <variant>

#include "nlohmann/json.hpp"
#include "salsa/behaviours/behaviour.h"
#include "salsa/behaviours/parameter.h"
#include "salsa/core/map.h"
#include "salsa/entity/drone_configuration.h"
#include "salsa/utils/base_contact_listener.h"
using json = nlohmann::json;

namespace salsa {

/// Represents the configuration for a single test within the simulation.
struct TestConfig {
  typedef std::unordered_map<std::string, behaviour::Parameter*> Parameters;
  typedef std::unordered_map<std::string, float> FloatParameters;

  std::string behaviour_name;
  std::variant<Parameters, FloatParameters> parameters;
  std::string drone_config_name;
  std::string map_name;
  int num_drones;
  int num_targets;
  float time_limit;
  std::string target_type = "null";
  std::string contact_listener_name;
  bool keep = true;
  // FUTURE: std::function<void()> drone_setup;
  // FUTURE: std::function<void()> target_setup;
};

void to_json(json& j, const TestConfig& config);

void from_json(const json& j, TestConfig& config);

/// Static class that manages a queue of test configurations.
class TestQueue {
 private:
  static std::vector<TestConfig> tests_;

 public:
  /// Adds a test configuration to the back of the test queue.
  /// @param test The test configuration to add.
  static void push(const TestConfig& test);

  /// Adds multiple permuted test configurations based on a base configuration.
  /// @param base The base test configuration.
  /// @param permutations The list of parameter value permutations.
  /// @param parameter_names The names of parameters corresponding to the
  /// permutations.
  static void addPermutedTests(
      const TestConfig& base,
      const std::vector<std::vector<float>>& permutations,
      const std::vector<std::string>& parameter_names);

  /// Removes and returns the first test configuration from the test queue.
  /// @return The first test configuration.
  /// @exception std::underflow_error Thrown if the queue is empty.
  static TestConfig pop();

  /// Returns the first test configuration from the test queue without removing
  /// it.
  /// @return The first test configuration.
  /// @exception std::underflow_error Thrown if the queue is empty.
  static TestConfig peek();

  static int size() { return tests_.size(); }

  static std::vector<TestConfig>& getTests() { return tests_; }

  static bool isEmpty() { return tests_.empty(); }

  static bool save(const std::string& filename);

  static bool load(const std::string& filename);
};

/// Loads permutations and parameter names from a JSON file.
/// @param permutations Vector to store the loaded permutations.
/// @param parameter_names Vector to store the loaded parameter names.
/// @param filename The name of the file to load from.
void loadPermutations(std::vector<std::vector<float>>& permutations,
                      std::vector<std::string>& parameter_names,
                      const std::string& filename);

}  // namespace salsa

#endif  // SWARM_SIM_CORE_TEST_STACK_H