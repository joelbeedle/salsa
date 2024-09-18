//
// Created by Joel Beedle on 17/09/2024.
//

#include "add_test_permutation_modal.h"
const auto behaviourNames = salsa::behaviour::Registry::get().behaviour_names();
const auto mapNames = salsa::map::getMapNames();

std::string AddTestPermutationModal::current_behaviour;
std::string AddTestPermutationModal::current_name = behaviourNames[0];
std::string AddTestPermutationModal::current_listener_name;
std::string AddTestPermutationModal::current_drone_config_name;
std::string AddTestPermutationModal::current_target_name;
std::string AddTestPermutationModal::current_map_name = mapNames[0];
int AddTestPermutationModal::new_drone_count = 0;
int AddTestPermutationModal::new_target_count = 0;
float AddTestPermutationModal::new_time_limit = 0.0f;

bool AddTestPermutationModal::to_change = false;

std::unordered_map<std::string, std::string> AddTestPermutationModal::input_storage;
std::vector<std::string> AddTestPermutationModal::parameter_names;
std::vector<int> AddTestPermutationModal::selections;
std::unordered_map<std::string, std::vector<float>> AddTestPermutationModal::range_storage;
bool AddTestPermutationModal::generated = false;
std::vector<salsa::TestConfig> AddTestPermutationModal::base;