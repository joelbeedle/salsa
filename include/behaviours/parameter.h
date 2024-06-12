/// @file parameter.h
/// @brief Contains the definition for the `Parameter` class.
#ifndef SWARM_SIM_BEHAVIOURS_PARAMETER_H
#define SWARM_SIM_BEHAVIOURS_PARAMETER_H

#include <iostream>
#include <stdexcept>

namespace swarm {
namespace behaviour {
/// @brief Represents a bounded numerical parameter, to be used for behaviors.
///
/// This class defines a parameter as a value that must be
/// maintained within specified minimum and maximum bounds. It includes
/// mechanisms to enforce these bounds.
///
/// Usage example:
/// ```cpp
/// Parameter parameter(0.5, 0.0, 1.0);
/// // We can access the value of the parameter directly:
/// parameter == 0.5; // true
/// parameter = 0.75;
/// // We can also get a reference to the value, if needed:
/// parameter.value() == 0.5; // false
/// ```
class Parameter {
  float value_;      ///< Current value of the parameter.
  float min_value_;  ///< Minimum allowable value for the parameter.
  float max_value_;  ///< Maximum allowable value for the parameter.

 public:
  /// @brief Constructs a Parameter with initial and boundary values.
  ///
  /// @param initial_value The initial value of the parameter.
  /// @param min_value The minimum value the parameter can take.
  /// @param max_value The maximum value the parameter can take.
  /// @throws std::out_of_range If the initial value is outside the [min_value,
  /// max_value] range.
  Parameter(float initial_value, float min_value, float max_value)
      : value_(initial_value), min_value_(min_value), max_value_(max_value) {
    if (initial_value < min_value || initial_value > max_value) {
      throw std::out_of_range("Initial value is out of the acceptable range.");
    }
  }

  /// @brief Assigns a new value to the parameter, ensuring it remains within
  /// bounds.
  ///
  /// @param newValue The new value to assign to the parameter.
  /// @return Reference to the updated value of the parameter.
  /// @details If newValue is out of bounds, an error message is printed and the
  /// value remains unchanged.
  float &operator=(const float &newValue) {
    if (newValue < min_value_ || newValue > max_value_) {
      std::cerr << "Assignment value out of bounds!" << std::endl;
      return value_;
    }
    value_ = newValue;
    return value_;
  }

  ///////////////////// Accessors and Mutators /////////////////////

  /// @brief Allows the parameter to be used as a float.
  /// @return The value of the parameter.
  operator float() const { return value_; }

  /// @brief Provides read and write access to the parameter's value.
  /// @return Reference to the value of the parameter.
  float &value() { return value_; }

  /// @brief Provides read access to the parameter's value.
  /// @return Constant reference to the value of the parameter.
  const float &value() const { return value_; }

  /// @brief Provides read and write access to the minimum allowable value of
  /// the parameter.
  /// @return Reference to the minimum value of the parameter.
  float &min_value() { return min_value_; }

  /// @brief Provides read access to the minimum allowable value of the
  /// parameter.
  /// @return Constant reference to the minimum value of the parameter.
  const float &min_value() const { return min_value_; }

  /// @brief Provides read and write access to the maximum allowable value of
  /// the parameter.
  /// @return Reference to the maximum value of the parameter.
  float &max_value() { return max_value_; }

  /// @brief Provides read access to the maximum allowable value of the
  /// parameter.
  /// @return Constant reference to the maximum value of the parameter.
  const float &max_value() const { return max_value_; }
};

}  // namespace behaviour
}  // namespace swarm

#endif  // SWARM_SIM_BEHAVIOURS_PARAMETER_H