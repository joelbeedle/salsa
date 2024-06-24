/// @file base_contact_listener.h
/// @brief Declares the BaseContactListener class for handling collision events

#ifndef SWARM_UTILS_BASE_CONTACT_LISTENER_H
#define SWARM_UTILS_BASE_CONTACT_LISTENER_H

#include <box2d/box2d.h>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <typeindex>

namespace swarm {

/// @class BaseContactListener
/// @brief Extends the Box2D contact listener to handle collision events between
/// registered types.
///
/// BaseContactListener provides a mechanism to dynamically register collision
/// handlers for pairs of object types at runtime. It utilizes std::map to store
/// function handlers for each unique pair of types, enabling custom responses
/// to collisions.
class BaseContactListener : public b2ContactListener {
 protected:
  /// @brief Maps pairs of types to their respective collision handlers.
  std::map<std::pair<std::string, std::string>,
           std::function<void(b2Fixture *, b2Fixture *)>>
      collision_handlers_;

 public:
  /// @brief Adds a collision handler for a specific pair of object types.
  ///
  /// @param type_a The std::type_index of the first object type.
  /// @param type_b The std::type_index of the second object type.
  /// @param handler The function to call when objects of type_a and type_b
  /// collide.
  void addCollisionHandler(
      std::string type_a, std::string type_b,
      std::function<void(b2Fixture *, b2Fixture *)> handler);

  /// @brief Override from b2ContactListener to handle the start of a contact
  /// event.
  ///
  /// This method is called when two fixtures begin to touch. It will
  /// check user data attached to fixtures and invoke the corresponding
  /// collision handler if registered.
  ///
  /// @param contact The contact point information about the collision.
  void BeginContact(b2Contact *contact) override;
};

}  // namespace swarm

#endif  // SWARM_UTILS_BASE_CONTACT_LISTENER_H