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
  static std::shared_ptr<spdlog::logger> logger_;

  // Private constructor
  Logger() { init_logger("default_log.log"); }

 public:
  // Deleted copy constructor and assignment operator
  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

  // Destructor
  ~Logger() { spdlog::shutdown(); }

  // Static method to get the logger instance
  static Logger& getInstance() {
    static Logger instance;
    return instance;
  }

  void init_logger(const std::string& log_file) {
    spdlog::drop("async_logger");
    spdlog::init_thread_pool(
        8192, 1);  // Queue with 8192 slots and 1 background thread
    logger_ = spdlog::basic_logger_mt<spdlog::async_factory>("async_logger",
                                                             log_file);
    logger_->set_pattern("%v");
    spdlog::set_default_logger(logger_);
    spdlog::set_level(spdlog::level::info);
    spdlog::flush_every(std::chrono::seconds(3));
  }

  void switch_log_file(const std::string& new_log_file) {
    init_logger(new_log_file);
  }

  void update(const nlohmann::json& message) override {
    float time = message["time"];
    std::string caller_info = message["caller_type"];
    int id = message["id"];
    std::string the_rest = message["message"];
    logger_->info("[{}] [{} {}] {}", time, caller_info, id, the_rest);
  }
};

}  // namespace swarm

#endif  // SWARM_CORE_DATA_H