#include <utility>

#include "salsa/core/logger.h"

#include <spdlog/sinks/basic_file_sink.h>

using namespace salsa;
// Initialize and manage the static logger
std::shared_ptr<spdlog::logger>& logger::get() {
  static std::shared_ptr<spdlog::logger> logger = spdlog::stdout_color_mt("salsa");
  return logger;
}

void logger::set(std::shared_ptr<spdlog::logger> custom_logger) {
  logger::get() = std::move(custom_logger);
}

void logger::log_info(const std::string& message) {
  logger::get()->info(message);
}

void logger::log_error(const std::string& message) {
  logger::get()->error(message);
}