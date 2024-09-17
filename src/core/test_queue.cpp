#include "salsa/core/test_queue.h"

#include "nlohmann/json.hpp"
#include "salsa/behaviours/registry.h"
namespace salsa {

std::vector<TestConfig> TestQueue::tests_;

void to_json(json& j, const TestConfig& config) {
  TestConfig::FloatParameters float_params;
  auto visitor = [&](auto&& arg) {
    float_params = Behaviour::convertParametersToFloat(arg);
  };
  std::visit(visitor, config.parameters);
  j = json({{"behaviour_name", config.behaviour_name},
            {"parameters", float_params},
            {"drone_config_name", config.drone_config_name},
            {"map_name", config.map_name},
            {"num_drones", config.num_drones},
            {"num_targets", config.num_targets},
            {"time_limit", config.time_limit},
            {"target_type", config.target_type},
            {"contact_listener_name", config.contact_listener_name},
            {"keep", config.keep}});
}

void from_json(const json& j, TestConfig& config) {
  j.at("behaviour_name").get_to(config.behaviour_name);
  if (j.at("parameters").contains("Parameters")) {
  } else if (j.at("parameters").contains("FloatParameters")) {
    config.parameters = j.at("parameters")["FloatParameters"]
                            .get<TestConfig::FloatParameters>();
  }
  j.at("drone_config_name").get_to(config.drone_config_name);
  j.at("map_name").get_to(config.map_name);
  j.at("num_drones").get_to(config.num_drones);
  j.at("num_targets").get_to(config.num_targets);
  j.at("time_limit").get_to(config.time_limit);
  j.at("target_type").get_to(config.target_type);
  j.at("contact_listener_name").get_to(config.contact_listener_name);
  j.at("keep").get_to(config.keep);
}

void TestQueue::push(const TestConfig& test) { tests_.push_back(test); }

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

bool TestQueue::save(const std::string& filename) {
  const json j = tests_;
  if (std::ofstream file(filename + ".json"); file.is_open()) {
    file << j.dump(4);
    file.close();
    return true;
  }
  return false;
}

bool TestQueue::load(const std::string& filename) {
  std::string full_filename = filename;

  // Check if the filename already ends with ".json"
  if (filename.size() < 5 || filename.substr(filename.size() - 5) != ".json") {
    full_filename += ".json";
  }

  if (std::ifstream file(full_filename); file.is_open()) {
    nlohmann::json j;
    file >> j;
    file.close();

    tests_ = j.get<std::vector<TestConfig>>();
    return true;
  }
  return false;
}

void TestQueue::addPermutedTests(
    const TestConfig& base, const std::vector<std::vector<float>>& permutations,
    const std::vector<std::string>& parameter_names) {
  for (const auto& combination : permutations) {
    std::unordered_map<std::string, salsa::behaviour::Parameter*> new_params;
    const TestConfig& modifiedConfig = base;  // Copy base config
    const auto chosen_behaviour =
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
                      const std::string& filename) {
  if (std::ifstream file(filename); file.is_open()) {
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