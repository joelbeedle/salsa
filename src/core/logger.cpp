#include "salsa/core/logger.h"

using namespace salsa;
// Initialize and manage the static logger
std::shared_ptr<spdlog::logger>& salsa::logger::get() {
  static std::shared_ptr<spdlog::logger> logger = spdlog::default_logger();
  return logger;
}

void logger::set(std::shared_ptr<spdlog::logger> custom_logger) {
  logger::get() = custom_logger;
}

void logger::log_info(const std::string& message) {
  logger::get()->info(message);
}

void logger::log_error(const std::string& message) {
  logger::get()->error(message);
}