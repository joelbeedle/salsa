#include "salsa/core/data.h"

#include "salsa/core/map.h"
using namespace salsa;

std::string salsa::generateRandomString(int length) {
  const std::string characters =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
  std::string randomString;

  for (int i = 0; i < length; ++i) {
    randomString += characters[rand() % characters.size()];
  }

  return randomString;
}

std::shared_ptr<spdlog::logger> Logger::logger_ = nullptr;

Logger::Logger() { init_logger("default_log.log"); }

Logger::~Logger() { spdlog::shutdown(); }

Logger& Logger::getInstance() {
  static Logger instance;
  return instance;
}

void Logger::init_logger(const std::string& log_file) {
  std::filesystem::path log_path = salsa::map::getExecutablePath() / ".." /
                                   ".." / "testbed" / "results" / log_file;
  spdlog::drop("async_logger");
  spdlog::init_thread_pool(8192,
                           1);  // Queue with 8192 slots and 1 background thread
  logger_ = spdlog::basic_logger_mt<spdlog::async_factory>("async_logger",
                                                           log_path.string());
  logger_->set_pattern("%v");
  spdlog::set_default_logger(logger_);
  spdlog::set_level(spdlog::level::info);
  spdlog::flush_every(std::chrono::seconds(3));
}

void Logger::switch_log_file(const std::string& new_log_file) {
  init_logger(new_log_file);
}

void Logger::update(const nlohmann::json& message) {
  float time = message["time"];
  std::string caller_info = message["caller_type"];
  int id = message["id"];
  std::string the_rest = message["message"];
  logger_->info("[{}] [{} {}] {}", time, caller_info, id, the_rest);
}
