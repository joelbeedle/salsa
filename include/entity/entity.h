#ifndef SWARM_ENTITY_H
#define SWARM_ENTITY_H

#include <box2d/box2d.h>

#include <vector>

#include "core/data.h"
namespace swarm {
class Entity {
 protected:
  b2Body *body_;
  b2World *world_;
  char id_prefix;
  int id;
  float radius_;
  std::vector<std::shared_ptr<Observer>> observers;
  std::chrono::steady_clock::time_point last_log_time;
  long log_interval;  // milliseconds

 public:
  Entity(b2World *world, const b2Vec2 &position, bool is_static, float radius,
         long log_interval = 1000.0)
      : world_(world), radius_(radius), log_interval(log_interval) {
    b2BodyDef bodyDef;
    if (is_static) {
      bodyDef.type = b2_staticBody;
    } else {
      bodyDef.type = b2_dynamicBody;
    }
    bodyDef.position = position;
    this->body_ = world_->CreateBody(&bodyDef);
    last_log_time = std::chrono::steady_clock::now();
  }

  Entity() = default;

  virtual ~Entity() {
    if (body_) {
      world_->DestroyBody(body_);
    }
  }

  virtual void create_fixture() = 0;

  float getRadius() { return radius_; }

  void setIdPrefix(char prefix) { id_prefix = prefix; }
  void setId(int id) { this->id = id; }

  long getLogInterval() { return log_interval; }
  void setLogInterval(long interval) { log_interval = interval; }

  void addObserver(std::shared_ptr<Observer> observer) {
    observers.push_back(observer);
  }

  void notifyAll(const nlohmann::json &message) {
    auto now = std::chrono::steady_clock::now();
    long duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - last_log_time)
                        .count();
    if (duration > log_interval) {
      nlohmann::json message_with_id = message;
      message_with_id["id"] = id_prefix + id;  // Prepend entity ID

      for (auto &observer : observers) {
        observer->update(message_with_id);
      }
      last_log_time = now;  // Update last log time
    }
  }
};

}  // namespace swarm

#endif  // SWARM_ENTITY_H
