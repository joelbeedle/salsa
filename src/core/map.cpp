#include "salsa/core/map.h"

namespace fs = std::filesystem;

using namespace swarm;
using namespace swarm::map;
using swarm::map::Map;

/// @brief Registry of all loaded maps
static std::vector<Map> registry;

fs::path swarm::map::getExecutablePath() {
#if defined(_WIN32)
  char path[MAX_PATH] = {0};
  HMODULE hModule = GetModuleHandle(nullptr);
  if (GetModuleFileNameA(hModule, path, MAX_PATH) == 0) {
    throw std::runtime_error("Failed to get module filename.");
  }
  return std::filesystem::path(path).parent_path();
#elif defined(__linux__)
  char result[PATH_MAX];
  ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
  if (count == -1) {
    throw std::runtime_error(
        "Failed to read symbolic link for executable path.");
  }
  return std::filesystem::path(std::string(result, (count > 0) ? count : 0))
      .parent_path();
#elif defined(__APPLE__)
  char path[1024];
  uint32_t size = sizeof(path);
  if (_NSGetExecutablePath(path, &size) != 0) {
    throw std::runtime_error("Buffer too small; increase buffer size.");
  }
  return std::filesystem::path(path).parent_path();
#else
  throw std::runtime_error("Unsupported platform.");
#endif
}

Map map::load(const char *new_map_name) {
  std::filesystem::path exec_path = getExecutablePath();
  swarm::logger::get()->info("Executable path: {}", exec_path.string());
  std::filesystem::path file_path = exec_path / ".." / ".." / "testbed" /
                                    "maps" /
                                    (std::string(new_map_name) + ".json");
  swarm::logger::get()->info("Loading map from: {}", file_path.string());

  b2World *world = new b2World(b2Vec2(0.0f, 0.0f));
  Map new_map;
  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file at: " + file_path.string());
  }

  nlohmann::json map;
  file >> map;
  new_map.name = map["name"];
  new_map.width = map["width"];
  new_map.height = map["height"];
  new_map.drone_spawn_point = {map["drone_spawn_point"][0],
                               map["drone_spawn_point"][1]};

  for (auto &body_json : map["bodies"]) {
    b2BodyDef body_def;
    body_def.type = (b2BodyType)body_json["type"];
    body_def.position.Set(body_json["position"][0], body_json["position"][1]);
    body_def.angle = body_json["angle"];
    body_def.linearDamping = body_json["linear_damping"];
    body_def.angularDamping = body_json["angular_damping"];
    body_def.gravityScale = body_json["gravity_scale"];
    body_def.fixedRotation = body_json["fixed_rotation"];
    body_def.bullet = body_json["bullet"];

    b2Body *body = world->CreateBody(&body_def);

    for (auto &fixture_json : body_json["fixtures"]) {
      b2FixtureDef fixture_def;
      fixture_def.density = fixture_json["density"];
      fixture_def.friction = fixture_json["friction"];
      fixture_def.restitution = fixture_json["restitution"];
      fixture_def.isSensor = fixture_json["is_sensor"];
      fixture_def.filter.categoryBits = fixture_json["category_bits"];
      fixture_def.filter.maskBits = fixture_json["mask_bits"];
      fixture_def.filter.groupIndex = fixture_json["group_index"];

      if (fixture_json.find("polygon") != fixture_json.end()) {
        b2PolygonShape shape;
        std::vector<b2Vec2> vertices;
        for (auto &vertex : fixture_json["polygon"]) {
          vertices.emplace_back(vertex[0], vertex[1]);
        }
        shape.Set(&vertices[0],
                  (int32)vertices.size());  // Properly set the vertices
        fixture_def.shape = &shape;
      } else if (fixture_json.find("circle") != fixture_json.end()) {
        b2CircleShape shape;
        shape.m_p.Set(fixture_json["circle"]["center"][0],
                      fixture_json["circle"]["center"][1]);
        shape.m_radius = fixture_json["circle"]["radius"];
        fixture_def.shape = &shape;
      } else if (fixture_json.find("edge") != fixture_json.end()) {
        b2EdgeShape shape;
        shape.m_vertex1.Set(fixture_json["edge"]["start"][0],
                            fixture_json["edge"]["start"][1]);
        shape.m_vertex2.Set(fixture_json["edge"]["end"][0],
                            fixture_json["edge"]["end"][1]);
        fixture_def.shape = &shape;
      }
      body->CreateFixture(&fixture_def);
    }
  }
  new_map.world = world;
  registry.push_back(new_map);
  return new_map;
}

void map::loadAll() {
  fs::path exec_path = getExecutablePath();
  fs::path directory = exec_path / ".." / ".." / "testbed" / "maps";
  if (!fs::exists(directory)) {
    throw std::runtime_error("Directory does not exist: " + directory.string());
  }
  for (const auto &entry : fs::directory_iterator(directory)) {
    if (entry.path().extension() == ".json") {
      std::string map_name = entry.path().stem().string();
      swarm::logger::get()->info("Loading map: {}", map_name);
      try {
        swarm::map::load(map_name.c_str());
      } catch (const std::exception &e) {
        swarm::logger::get()->error("Failed to load map: {}", map_name);
        swarm::logger::get()->error("Error: {}", e.what());
      }
    }
  }
}

std::vector<std::string> map::getMapNames() {
  std::vector<std::string> names;
  for (const auto &map : registry) {
    names.push_back(map.name);
  }
  return names;
}

Map map::getMap(const std::string &name) {
  for (const auto &map : registry) {
    if (map.name == name) {
      return map;
    }
  }
  throw std::runtime_error("Map not found: " + name);
}

void map::save(Map new_map) {
  std::cout << "Saving Map: " << new_map.name << "\n";
  // PrintBodiesAndFixtures(m_world);
  // Save map
  nlohmann::json map;
  map["name"] = new_map.name;
  map["width"] = new_map.width;
  map["height"] = new_map.height;
  map["drone_spawn_point"] = {new_map.drone_spawn_point.x,
                              new_map.drone_spawn_point.y};
  for (b2Body *body = new_map.world->GetBodyList(); body;
       body = body->GetNext()) {
    nlohmann::json body_json;
    body_json["type"] = body->GetType();
    body_json["position"] = {body->GetPosition().x, body->GetPosition().y};
    body_json["angle"] = body->GetAngle();
    body_json["linear_damping"] = body->GetLinearDamping();
    body_json["angular_damping"] = body->GetAngularDamping();
    body_json["gravity_scale"] = body->GetGravityScale();
    body_json["fixed_rotation"] = body->IsFixedRotation();
    body_json["bullet"] = body->IsBullet();

    for (b2Fixture *fixture = body->GetFixtureList(); fixture;
         fixture = fixture->GetNext()) {
      nlohmann::json fixture_json;
      fixture_json["density"] = fixture->GetDensity();
      fixture_json["friction"] = fixture->GetFriction();
      fixture_json["restitution"] = fixture->GetRestitution();
      fixture_json["is_sensor"] = fixture->IsSensor();
      fixture_json["category_bits"] = fixture->GetFilterData().categoryBits;
      fixture_json["mask_bits"] = fixture->GetFilterData().maskBits;
      fixture_json["group_index"] = fixture->GetFilterData().groupIndex;

      b2Shape *shape = fixture->GetShape();
      if (shape->GetType() == b2Shape::e_polygon) {
        b2PolygonShape *polygon = (b2PolygonShape *)shape;
        nlohmann::json polygon_json;
        for (int i = 0; i < polygon->m_count; ++i) {
          polygon_json.push_back(
              {polygon->m_vertices[i].x, polygon->m_vertices[i].y});
        }
        fixture_json["polygon"] = polygon_json;
      } else if (shape->GetType() == b2Shape::e_circle) {
        b2CircleShape *circle = (b2CircleShape *)shape;
        nlohmann::json circle_json;
        circle_json["center"] = {circle->m_p.x, circle->m_p.y};
        circle_json["radius"] = circle->m_radius;
        fixture_json["circle"] = circle_json;
      } else if (shape->GetType() == b2Shape::e_edge) {
        b2EdgeShape *edge = (b2EdgeShape *)shape;
        nlohmann::json edge_json;
        edge_json["start"] = {edge->m_vertex1.x, edge->m_vertex1.y};
        edge_json["end"] = {edge->m_vertex2.x, edge->m_vertex2.y};
        fixture_json["edge"] = edge_json;
      }
      body_json["fixtures"].push_back(fixture_json);
    }
    map["bodies"].push_back(body_json);
  }
  fs::path exec_path = getExecutablePath();
  fs::path directory = exec_path / ".." / ".." / "testbed" / "maps";
  fs::path file_path = directory / (std::string(new_map.name) + ".json");

  if (!fs::exists(directory)) {
    // Try to create the directory
    if (!fs::create_directories(directory)) {
      std::cerr << "Failed to create directory: " << directory << std::endl;
      return;
    }
  }

  std::ofstream file(file_path);
  if (!file.is_open() || file.fail()) {
    std::cerr << "Failed to open file for writing: " << file_path << std::endl;
    return;
  }

  file << map.dump(4);
  if (file.fail()) {
    std::cerr << "Failed to write to file: " << file_path << std::endl;
  } else {
    std::cout << "World saved successfully to " << file_path << std::endl;
  }
  file.close();
}