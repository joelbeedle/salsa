#ifndef SWARM_SIM_BEHAVIOURS_PARAMETER_H
#define SWARM_SIM_BEHAVIOURS_PARAMETER_H

#include <iostream>
#include <stdexcept>

namespace swarm {
namespace behaviour {
class Parameter {
  float value_;
  float min_value_;
  float max_value_;

 public:
  // Constructor to initialize a parameter with a value and constraints
  Parameter(float initial_value, float min_value, float max_value)
      : value_(initial_value), min_value_(min_value), max_value_(max_value) {
    if (initial_value < min_value || initial_value > max_value) {
      throw std::out_of_range("Initial value is out of the acceptable range.");
    }
  }

  float& value() { return value_; }
  const float& value() const { return value_; }

  float& min_value() { return min_value_; }
  const float& min_value() const { return min_value_; }

  float& max_value() { return max_value_; }
  const float& max_value() const { return max_value_; }

  // Overload assignment to ensure the value remains within bounds
  float& operator=(const float& newValue) {
    if (newValue < min_value_ || newValue > max_value_) {
      std::cerr << "Assignment value out of bounds!" << std::endl;
      return value_;
    }
    value_ = newValue;
    return value_;
  }

  // Implicit conversion to float
  operator float() const { return value_; }
};
}  // namespace behaviour
}  // namespace swarm

#endif  // SWARM_SIM_BEHAVIOURS_PARAMETER_H