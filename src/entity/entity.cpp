#include <utility>

#include "salsa/entity/entity.h"

salsa::Entity::Entity(b2World *world, const b2Vec2 &position, const bool is_static,
                      const float radius, std::string type_name, const long log_interval)
    : world_(world),
      radius_(radius),
      color_(b2Color(0.5, 0.5, 0.5)),
      type_name_(std::move(type_name)),
      log_interval_(log_interval) {
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
struct fmt::formatter<salsa::Entity> : fmt::formatter<std::string> {
  static auto format(const salsa::Entity& my, format_context &ctx) -> decltype(ctx.out()) {
    return fmt::format_to(ctx.out(), "[{} {}]", salsa::type(my), my.id());
  }
};

void salsa::Entity::notifyAll(float time, const nlohmann::json &message) {
  nlohmann::json message_with_id;
  message_with_id["message"] = message.dump();
  message_with_id["time"] = time;
  message_with_id["id"] = id_;
  message_with_id["caller_type"] = fmt::format("{}", type_name_);
  for (const auto &observer : observers) {
    observer->update(message_with_id);
  }
}
