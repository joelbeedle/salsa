//
// Created by Joel Beedle on 17/09/2024.
//

#include "add_test_modal.h"
std::unordered_map<std::string, salsa::behaviour::Parameter *> AddTestModal::new_params;
const auto behaviourNames = salsa::behaviour::Registry::get().behaviour_names();
const auto mapNames = salsa::map::getMapNames();

std::string AddTestModal::current_behaviour;
std::string AddTestModal::current_name = behaviourNames[0];
std::string AddTestModal::current_target_name;
std::string AddTestModal::current_listener_name;
std::string AddTestModal::current_drone_config_name;
std::string AddTestModal::current_map_name = mapNames[0];
int AddTestModal::drone_count = 0;
int AddTestModal::target_count = 0;
float AddTestModal::time_limit = 0.0;
float AddTestModal::new_time_limit = 0.0;
b2World* AddTestModal::world = nullptr;
bool AddTestModal::to_change = false;
