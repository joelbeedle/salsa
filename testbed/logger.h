//
// Created by Joel Beedle on 18/09/2024.
//
#ifndef TESTBED_LOGGER_H
#define TESTBED_LOGGER_H

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <salsa/salsa.h>
#include <memory>

#include "imgui_logger.h"

namespace testbed {
// Declare a global logger shared pointer that will be accessible across files
extern std::shared_ptr<spdlog::logger> logger;
extern std::shared_ptr<ImGuiLoggerSink> imgui_logger_sink;
// Function to set up the logger
void setupLogger();
};

#ifndef TESTBED_FUNCTION
#define TESTBED_FUNCTION static_cast<const char *>(__FUNCTION__)
#endif

#define LOGGER_CALL(logger, level, ...) \
(logger)->log(spdlog::source_loc{__FILE__, __LINE__, TESTBED_FUNCTION}, level, __VA_ARGS__)

#define LOGGER_INFO(logger, ...) \
  LOGGER_CALL(logger, spdlog::level::info, __VA_ARGS__)
#define LOG_INFO(...) if(testbed::logger) LOGGER_INFO(testbed::logger, __VA_ARGS__)

#define LOGGER_WARN(logger, ...) \
  LOGGER_CALL(logger, spdlog::level::warn, __VA_ARGS__)
#define LOG_WARN(...) if(testbed::logger) LOGGER_WARN(testbed::logger, __VA_ARGS__)

#define LOGGER_ERROR(logger, ...) \
  LOGGER_CALL(logger, spdlog::level::err, __VA_ARGS__)
#define LOG_ERROR(...) if(testbed::logger) LOGGER_ERROR(testbed::logger, __VA_ARGS__)

#define LOGGER_DEBUG(logger, ...) \
  LOGGER_CALL(logger, spdlog::level::debug, __VA_ARGS__)
#define LOG_DEBUG(...) if(testbed::logger) LOGGER_DEBUG(testbed::logger, __VA_ARGS__)

#endif // TESTBED_LOGGER_H