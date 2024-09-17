/// @file base_contact_listener.h
/// @brief Defines the BaseContactListener class for handling collision events
#ifndef SWARM_UTILS_BASE_CONTACT_LISTENER_H
#define SWARM_UTILS_BASE_CONTACT_LISTENER_H

#include <box2d/box2d.h>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <typeindex>

namespace salsa {

/// @class BaseContactListener
/// @brief Extends the Box2D contact listener to handle collision events between
/// registered types.
///
/// Provides a mechanism to dynamically register collision handlers for pairs of
/// object types at runtime, enabling custom responses to collisions.
class BaseContactListener : public b2ContactListener {
 protected:
  /// @brief Maps pairs of types to their respective collision handlers.
  std::map<std::pair<std::string, std::string>,
           std::function<void(b2Fixture *, b2Fixture *)>>
      collision_handlers_;

  std::string name_;
  static std::vector<BaseContactListener *> registry_;

 public:
  explicit BaseContactListener(std::string name);

  /// @brief Destructor to remove this listener from the registry.
  virtual ~BaseContactListener();

  /// @brief Registers a collision handler for a specific pair of object types.
  /// @param type1 Name of the first object type.
  /// @param type_b Name of the second object type.
  /// @param handler The function to call when objects of type_a and type_b
  /// collide.
  void addCollisionHandler(
      std::string type1, std::string type2,
      const std::function<void(b2Fixture*, b2Fixture*)> &handler);

  /// @brief Called when two fixtures begin to touch.
  /// @param contact The contact point information about the collision.
  void BeginContact(b2Contact *contact) override;
  static const std::vector<BaseContactListener *> &registry() {
    return registry_;
  }

  /// @brief Retrieves the names of all listeners in the registry.
  /// @return A vector of names of all registered listeners.
  static std::vector<std::string> getListenerNames();

  /// @brief Finds a listener by its name.
  /// @param name The name of the listener to find.
  /// @return Pointer to the listener, or nullptr if not found.
  static BaseContactListener *getListenerByName(const std::string &name);

  /// @brief Accessor for the listener's name.
  /// @return The name of the listener.
  const std::string &name() const {
    { return name_; }
  }
};

}  // namespace salsa

#endif  // SWARM_UTILS_BASE_CONTACT_LISTENER_H