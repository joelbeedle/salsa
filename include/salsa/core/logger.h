/// @file logger.h
/// @brief Defines the logger utility functions for SALSA.
#ifndef LOGGER_H
#define LOGGER_H

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <memory>

namespace salsa {
namespace logger {

// Declaration of logger accessor and setter
std::shared_ptr<spdlog::logger>& get();
void set(std::shared_ptr<spdlog::logger> custom_logger);

// Logging utility functions
void log_info(const std::string& message);
void log_error(const std::string& message);
}  // namespace logger
}  // namespace salsa
#endif  // LOGGER_H
