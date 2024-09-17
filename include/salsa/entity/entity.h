/// @file entity.h
/// @brief Contains the `Entity` class, which is the base class for all physical
/// entities.
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
namespace salsa {
/// @class Entity
/// @brief Represents a generic physical entity in a simulation environment.
///
/// Provides basic functionality for all derived physical objects, such as
/// bodies in a physics simulation, with properties like position, radius,
/// and color, and the ability to log events and notify observers.
class Entity {
 protected:
  b2Body *body_;    ///< Pointer to the Box2D body associated with this entity.
  b2World *world_;  ///< Pointer to the Box2D world in which this entity exists.
  int id_{};          ///< Numeric identifier for the entity.
  float radius_;    ///< Radius of the entity, used for collision and rendering.
  b2Color color_;   ///< Color of the entity, used for rendering.
  std::string type_name_;  ///< String representing the type of the entity.
  std::vector<std::shared_ptr<Observer>>
      observers;  ///< List of observers to notify on updates.
  std::chrono::steady_clock::time_point
      last_log_time;   ///< Last time a log was recorded.
  long log_interval_;  ///< Interval at which to log information (in
                       ///< milliseconds).

 public:
  /// @brief Constructor for initializing an entity in the world.
  /// @param world Pointer to the b2World.
  /// @param position Initial position of the entity.
  /// @param is_static Flag indicating if the entity is static.
  /// @param radius Radius of the entity.
  /// @param type_name Name of the entity type.
  /// @param log_interval Interval for logging updates (default 100ms).
  Entity(b2World *world, const b2Vec2 &position, bool is_static, float radius,
         std::string type_name, long log_interval = 100.0);

  Entity() = default;

  /// @brief Destructor to handle clean-up.
  virtual ~Entity() {
    if (body_) {
      world_->DestroyBody(body_);
    }
  }

  /// @brief Adds an observer to the list of entity observers.
  /// @param observer Shared pointer to the observer to add.
  void addObserver(std::shared_ptr<Observer> observer) {
    observers.push_back(observer);
  }

  /// @brief Notifies all observers with a given message.
  /// @param time Current simulation time.
  /// @param message JSON formatted message with the notification details.
  void notifyAll(float time, const nlohmann::json &message);

  /// @name Getters and Setters
  /// @{
  float radius() const { return radius_; }
  void radius(float new_radius) { radius_ = new_radius; }

  b2Vec2 position() const { return body_->GetPosition(); }

  int id() const { return id_; }
  void id(int new_id) { id_ = new_id; }

  b2Color color() const { return color_; }
  void color(b2Color new_color) { color_ = new_color; }

  long log_interval() const { return log_interval_; }
  void log_interval(long interval) { log_interval_ = interval; }
  ///@}
};

}  // namespace salsa

#endif  // SWARM_ENTITY_H
