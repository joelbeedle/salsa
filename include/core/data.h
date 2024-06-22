#ifndef SWARM_CORE_DATA_H
#define SWARM_CORE_DATA_H

#include <box2d/box2d.h>

#include <memory>

#include "nlohmann/json.hpp"
#include "spdlog/async.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/spdlog.h"

namespace swarm {

class Observer {
 public:
  virtual ~Observer() {}
  virtual void update(const nlohmann::json& message) = 0;
};

class Logger : public Observer {
 private:
  static std::shared_ptr<spdlog::logger> async_logger;
  static std::shared_ptr<Logger> instance;
  static std::mutex mutex;

  // Private constructor
  Logger(const std::string& log_filename) {
    if (!async_logger) {  // Ensure the logger is only initialized once
      spdlog::init_thread_pool(
          8192, 1);  // Queue with 8192 slots and 1 background thread
      async_logger = spdlog::basic_logger_mt<spdlog::async_factory>(
          "async_logger", log_filename);
      spdlog::set_default_logger(async_logger);
      spdlog::set_level(spdlog::level::info);
      spdlog::flush_every(std::chrono::seconds(3));
    }
  }

 public:
  // Deleted copy constructor and assignment operator
  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

  // Destructor
  ~Logger() { spdlog::shutdown(); }

  // Static method to get the logger instance
  static std::shared_ptr<Logger> getInstance(const std::string& log_filename) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!instance) {
      instance = std::shared_ptr<Logger>(new Logger(log_filename));
    }
    return instance;
  }

  void update(const nlohmann::json& message) override {
    async_logger->info(message.dump());
  }
};

}  // namespace swarm

#endif  // SWARM_CORE_DATA_H