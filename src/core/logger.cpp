#include "salsa/core/logger.h"

using namespace swarm;
// Initialize and manage the static logger
std::shared_ptr<spdlog::logger>& swarm::logger::get() {
  static std::shared_ptr<spdlog::logger> logger = spdlog::default_logger();
  return logger;
}

void set(std::shared_ptr<spdlog::logger> custom_logger) {
  logger::get() = custom_logger;
}

void log_info(const std::string& message) { logger::get()->info(message); }

void log_error(const std::string& message) { logger::get()->error(message); }