//
// Created by Joel Beedle on 18/09/2024.
//

#include <iostream>
#include <spdlog/spdlog.h>
#include <salsa/salsa.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "imgui_logger.h"

class ImGuiLoggerSink;

namespace testbed {
std::shared_ptr<spdlog::logger> logger;
std::shared_ptr<ImGuiLoggerSink> imgui_logger_sink = std::make_shared<ImGuiLoggerSink>();
void setupLogger() {
  try {
    // Create the console logger sink
    const auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

    // Create the ImGui logger sink
    imgui_logger_sink = std::make_shared<ImGuiLoggerSink>();

    // Combine both sinks into a single logger
    std::vector<spdlog::sink_ptr> sinks {console_sink, imgui_logger_sink};
    logger = std::make_shared<spdlog::logger>("testbed", begin(sinks), end(sinks));

    const auto salsa_logger = salsa::logger::get();  // Get salsa logger instance
    salsa_logger->sinks().push_back(imgui_logger_sink);  // Add ImGui sink to salsa logger

    spdlog::info("Logger initialized with console and ImGui sinks.");
  } catch (const spdlog::spdlog_ex &ex) {
    std::cerr << "Log initialization failed: " << ex.what() << std::endl;
  }
}

};

