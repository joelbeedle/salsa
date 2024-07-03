/// @file data.h
/// @brief Contains the `Observer` and `Logger` classes for logging simulation
/// data.
#ifndef SWARM_CORE_DATA_H
#define SWARM_CORE_DATA_H

#include <box2d/box2d.h>

#include <memory>

#include "nlohmann/json.hpp"
#include "spdlog/async.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/spdlog.h"

namespace salsa {

std::string generateRandomString(int length);

/// @brief Abstract base class for an Observer in the Observer pattern.
class Observer {
 public:
  virtual ~Observer() {}

  /// @brief Pure virtual function to update the observer with a JSON message
  /// @param message A JSON object containing the update data.
  virtual void update(const nlohmann::json& message) = 0;
};

/// @brief Singleton Logger class for asynchronous simulation logging.
class Logger : public Observer {
 private:
  static std::shared_ptr<spdlog::logger>
      logger_;  ///< Static logger shared across instances.

  /// @brief Private constructor for the Logger class.
  Logger();

  /// @brief Initialise or re-initialize the logger with a specified file sink
  /// @param log_file The file to output log data to
  void init_logger(const std::string& log_file);

 public:
  // Delete copy constructor and assignment operator
  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;

  /// @brief Destructor for Logger class. Shuts down the logger.
  ~Logger();

  /// @brief Get the singleton instance of the logger
  /// @return Reference to the singleton instance of the logger
  static Logger& getInstance();

  /// @brief Switches the file to log into. Closes the current file and logger
  /// and creates a new logger, with its output set to the new file.
  ///
  /// Internally, just calls `init_logger` with the new log file.
  /// @param new_log_file
  void switch_log_file(const std::string& new_log_file);

  /// @brief Update the log with a new message.
  ///
  /// `message` is expected to contain the keys:
  ///   - 'time': A float representing the time of the message.
  ///   - 'caller_type': A string representing the type of the caller.
  ///   - 'id': An integer representing the ID of the caller.
  ///   - 'message': A string representing the message to log.
  /// @param message A JSON object containing the keys 'time', 'caller_type',
  /// 'id' and 'message.'.
  void update(const nlohmann::json& message) override;
};

}  // namespace salsa

#endif  // SWARM_CORE_DATA_H