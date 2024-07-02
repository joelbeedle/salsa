/// @file object_types.h
/// @brief Defines the ObjectType enum class and UserData struct.
#ifndef SWARM_SIM_UTILS_OBJECT_TYPES_H
#define SWARM_SIM_UTILS_OBJECT_TYPES_H
#include <iostream>
namespace salsa {
class Entity;

/// @brief Enum class representing different types of objects in the simulation.
enum class ObjectType {
  Drone,   ///< Represents a drone object.
  Target,  ///< Represents a target object.
};

/// @brief Demangles the name of a type for more readable output.
/// @param name The mangled name of the type.
/// @return The demangled name as a string.
std::string demangle(const char *name);

/// @brief Gets the demangled type name of a given object.
/// @tparam T The type of the object.
/// @param t The object to get the type name of.
/// @return The demangled type name as a string.
template <class T>
std::string type(const T &t) {
  return demangle(typeid(t).name());
}

/// @brief Gets the demangled type name of a type.
/// @tparam T The type to get the name of.
/// @return The demangled type name as a string.
template <typename T>
std::string get_type() {
  return demangle(typeid(T).name());
}

/// @brief Struct for storing user data associated with an entity.
///
/// This struct can store a pointer to any object derived from Entity and
/// provides a method to safely cast the stored object to a specific type.
struct UserData {
  Entity *object;  ///< Pointer to an object derived from Entity.

  /// @brief Default constructor initializing with a nullptr.
  UserData() : object(nullptr) {}

  /// @brief Constructor initializing with a specific object.
  /// @param obj Pointer to the object to store.
  explicit UserData(Entity *obj) : object(obj) {}

  /// @brief Safely casts the stored object to the requested type.
  /// @tparam T The type to cast the object to.
  /// @return Pointer to the object cast to the requested type.
  template <typename T>
  T *as() {
    return static_cast<T *>(object);
  }
};

}  // namespace salsa

#endif  // SWARM_SIM_UTILS_OBJECT_TYPES_H