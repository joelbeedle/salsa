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
  std::shared_ptr<spdlog::logger> async_logger;

 public:
  Logger(const std::string& log_filename) {
    spdlog::init_thread_pool(
        8192, 1);  // Queue with 8192 slots and 1 background thread
    async_logger = spdlog::basic_logger_mt<spdlog::async_factory>(
        "async_logger", log_filename);
    spdlog::set_default_logger(async_logger);
    spdlog::set_level(spdlog::level::info);
    spdlog::flush_every(std::chrono::seconds(3));
  }

  void update(const nlohmann::json& message) override {
    async_logger->info(message.dump());
  }
};
}  // namespace swarm

#endif  // SWARM_CORE_DATA_H