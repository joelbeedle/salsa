#include "core/data.h"

namespace swarm {
std::shared_ptr<spdlog::logger> Logger::async_logger = nullptr;
std::shared_ptr<Logger> Logger::instance = nullptr;
std::mutex Logger::mutex;
};  // namespace swarm