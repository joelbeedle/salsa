#include "salsa/entity/entity.h"

swarm::Entity::Entity(b2World *world, const b2Vec2 &position, bool is_static,
                      float radius, std::string type_name, long log_interval)
    : world_(world),
      radius_(radius),
      log_interval_(log_interval),
      type_name_(type_name),
      color_(b2Color(0.5, 0.5, 0.5)) {
  b2BodyDef bodyDef;
  if (is_static) {
    bodyDef.type = b2_staticBody;
  } else {
    bodyDef.type = b2_dynamicBody;
  }
  bodyDef.position = position;
  body_ = world_->CreateBody(&bodyDef);
  last_log_time = std::chrono::steady_clock::now();
}

template <>
struct fmt::formatter<swarm::Entity> : fmt::formatter<std::string> {
  auto format(swarm::Entity my, format_context &ctx) -> decltype(ctx.out()) {
    return fmt::format_to(ctx.out(), "[{} {}]", swarm::type(my), my.id());
  }
};

void swarm::Entity::notifyAll(float time, const nlohmann::json &message) {
  nlohmann::json message_with_id;
  message_with_id["message"] = message.dump();
  message_with_id["time"] = time;
  message_with_id["id"] = id_;
  message_with_id["caller_type"] = fmt::format("{}", type_name_);
  for (auto &observer : observers) {
    observer->update(message_with_id);
  }
}
