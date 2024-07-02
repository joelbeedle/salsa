#include "salsa/core/test_queue.h"

#include "nlohmann/json.hpp"
#include "salsa/behaviours/registry.h"
namespace salsa {

std::vector<TestConfig> TestQueue::tests_;

void TestQueue::push(TestConfig test) { tests_.push_back(test); }

TestConfig TestQueue::pop() {
  if (tests_.empty()) {
    throw std::underflow_error("Cannot pop from an empty queue");
  }
  TestConfig test = tests_.front();
  tests_.erase(tests_.begin());
  return test;
}

TestConfig TestQueue::peek() {
  if (tests_.empty()) {
    throw std::underflow_error("Cannot peek from an empty queue");
  }
  return tests_.front();
}

void TestQueue::addPermutedTests(
    const TestConfig& base, const std::vector<std::vector<float>>& permutations,
    const std::vector<std::string>& parameter_names) {
  for (const auto& combination : permutations) {
    std::unordered_map<std::string, salsa::behaviour::Parameter*> new_params;
    TestConfig modifiedConfig = base;  // Copy base config
    auto chosen_behaviour =
        salsa::behaviour::Registry::get().behaviour(base.behaviour_name);
    auto chosen_params = chosen_behaviour->getParameters();
    for (size_t j = 0; j < parameter_names.size(); ++j) {
      const auto& name = parameter_names[j];
      new_params[name] = chosen_params[name]->clone();
      *(new_params[name]) = static_cast<float>(combination[j]);
    }

    TestQueue::push(modifiedConfig);
  }
}

void loadPermutations(std::vector<std::vector<float>>& permutations,
                      std::vector<std::string>& parameter_names,
                      std::string filename) {
  std::ifstream file(filename);
  if (file.is_open()) {
    nlohmann::json j;
    file >> j;
    file.close();

    permutations = j["permutations"].get<std::vector<std::vector<float>>>();
    parameter_names = j["parameter_names"].get<std::vector<std::string>>();
  } else {
    std::cout << "Failed to open " << filename << std::endl;
  }
}

}  // namespace salsa