#ifndef SWARM_TESTBED_MAP_H
#define SWARM_TESTBED_MAP_H

#include <box2d/box2d.h>

#include <fstream>
#include <iostream>

#include "nlohmann/json.hpp"
#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <limits.h>
#include <unistd.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
namespace swarm {
namespace map {
struct Map {
  std::string name;
  float width;
  float height;
  b2Vec2 drone_spawn_point;
  b2World *world;
};

Map load(const char *new_map_name);
void save(Map map);
std::filesystem::path getExecutablePath();
}  // namespace map
}  // namespace swarm
#endif  // SWARM_TESTBED_MAP_H