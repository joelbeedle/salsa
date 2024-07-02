#ifndef SWARM_ENTITY_H
#define SWARM_ENTITY_H

#include <box2d/box2d.h>

#include <chrono>
#include <cstdio>
#include <vector>

#include "salsa/core/data.h"
#include "salsa/utils/object_types.h"
#include "spdlog/cfg/env.h"  // support for loading levels from the environment variable
#include "spdlog/fmt/ostr.h"  // support for user defined types
#include "spdlog/spdlog.h"
namespace swarm {
class Entity {
 protected:
  b2Body *body_;
  b2World *world_;
  char id_prefix;
  int id;
  float radius_;
  b2Color color_;
  std::string type_name_;
  std::vector<std::shared_ptr<Observer>> observers;
  std::chrono::steady_clock::time_point last_log_time;
  long log_interval;  // milliseconds

 public:
  Entity(b2World *world, const b2Vec2 &position, bool is_static, float radius,
         std::string type_name, long log_interval = 100.0);

  Entity() = default;

  virtual ~Entity() {
    if (body_) {
      world_->DestroyBody(body_);
    }
  }

  float getRadius() { return radius_; }
  void setIdPrefix(char prefix) { id_prefix = prefix; }
  void setId(int id) { this->id = id; }
  void setColor(b2Color color) { color_ = color; }
  b2Color getColor() { return color_; }
  int getId() { return id; }
  long getLogInterval() { return log_interval; }
  void setLogInterval(long interval) { log_interval = interval; }

  void addObserver(std::shared_ptr<Observer> observer) {
    observers.push_back(observer);
  }

  void notifyAll(float time, const nlohmann::json &message);
};

}  // namespace swarm

#endif  // SWARM_ENTITY_H
