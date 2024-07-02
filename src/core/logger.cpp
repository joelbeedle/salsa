#include "salsa/core/logger.h"

namespace swarm {
// Initialize and manage the static logger
std::shared_ptr<spdlog::logger>& get_logger() {
  static std::shared_ptr<spdlog::logger> logger = spdlog::default_logger();
  return logger;
}

void set_logger(std::shared_ptr<spdlog::logger> custom_logger) {
  get_logger() = custom_logger;
}

void log_info(const std::string& message) { get_logger()->info(message); }

void log_error(const std::string& message) { get_logger()->error(message); }
}  // namespace swarm