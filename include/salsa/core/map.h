/// @file map.h
/// @brief Contains the map struct and functions to load and save maps
#ifndef SWARM_TESTBED_MAP_H
#define SWARM_TESTBED_MAP_H

#include <box2d/box2d.h>

#include <exception>
#include <fstream>
#include <iostream>
#include <string>

#include "logger.h"
#include "nlohmann/json.hpp"
#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <limits.h>
#include <unistd.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
namespace salsa {
namespace map {
/// @brief Struct to hold map data
struct Map {
  std::string name;
  float width;
  float height;
  b2Vec2 drone_spawn_point;
  b2World *world;
};

/// @brief Gets the absolute path to the executable, on WIN32, Linux, and Apple
/// systems.
/// @return The absolute path to the executable.
std::filesystem::path getExecutablePath();

/// @brief Loads a map from a JSON file. The map must be stored in
/// `testbed/maps`
/// @param new_map_name The map name of the json file, excluding .json
/// @return A struct created from parsing the JSON file.
Map load(const char *new_map_name);

/// @brief Loads all maps from the `testbed/maps` directory.
void loadAll();

/// @brief Saves a map to a JSON file in the `testbed/maps` directory.
/// @param map The map to save.
void save(Map map);

/// @brief Gets a list of all map names in the `testbed/maps` directory, if they
/// have been loaded using `map::loadAll`.
/// @return A vector of strings containing all map names.
std::vector<std::string> getMapNames();

/// @brief Gets a map by name from the map registry, if it has been loaded.
/// @param name The name of the map to retrieve.
/// @return The map with the given name.
Map getMap(const std::string &name);

}  // namespace map
}  // namespace salsa
#endif  // SWARM_TESTBED_MAP_H