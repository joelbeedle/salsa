#include "entity/entity.h"

swarm::Entity::Entity(b2World *world, const b2Vec2 &position, bool is_static,
                      float radius, std::string type_name, long log_interval)
    : world_(world),
      radius_(radius),
      log_interval(log_interval),
      type_name_(type_name) {
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

template <>
struct fmt::formatter<swarm::Entity> : fmt::formatter<std::string> {
  auto format(swarm::Entity my, format_context &ctx) -> decltype(ctx.out()) {
    return fmt::format_to(ctx.out(), "[{} {}]", swarm::type(my), my.getId());
  }
};

void swarm::Entity::notifyAll(const nlohmann::json &message) {
  auto now = std::chrono::steady_clock::now();
  long duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time)
          .count();
  if (duration > log_interval) {
    nlohmann::json message_with_id;
    message_with_id["message"] = message.dump();
    message_with_id["id"] = id;
    message_with_id["caller_type"] = fmt::format("{}", type_name_);
    for (auto &observer : observers) {
      observer->update(message_with_id);
    }
    last_log_time = now;
  }
}
