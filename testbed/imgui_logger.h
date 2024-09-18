//
// Created by Joel Beedle on 18/09/2024.
//

#ifndef IMGUI_LOGGER_H
#define IMGUI_LOGGER_H

#include <spdlog/sinks/base_sink.h>
#include <imgui.h>

#include <mutex>
#include <deque>


// Custom ImGui logger sink
class ImGuiLoggerSink final : public spdlog::sinks::base_sink<std::mutex> {
public:
  // Store the log messages in a buffer
  struct LogEntry {
    std::string message;
    std::string time;
    std::string logger_name;
    spdlog::level::level_enum level;
  };

  // Store the log messages in a buffer
  std::deque<LogEntry> log_messages;

  ImGuiTextBuffer Buf;
  ImGuiTextFilter Filter;
  ImVector<int> LineOffsets;
  // Index to lines offset. We maintain this with AddLog() calls.
  bool AutoScroll; // Keep scrolling if already at the bottom.
  bool is_open = true;

protected:
  // Override the sink's log function to capture messages
  void sink_it_(const spdlog::details::log_msg& msg) override {
    // Format the message using spdlog's formatter
    spdlog::memory_buf_t formatted;
    const auto duration_since_epoch = msg.time.time_since_epoch();
    const auto seconds_since_epoch = std::chrono::duration_cast<std::chrono::seconds>(duration_since_epoch);
    std::time_t log_time = seconds_since_epoch.count();

    // Convert to a human-readable format (e.g., "2024-09-18 13:22:34")
    std::tm tm_buf{};
    #if defined(_WIN32) || defined(_WIN64)
      localtime_r(&log_time, &tm_buf);  // Convert to local time
    #else
      localtime_r(&log_time, &tm_buf);  // Convert to local time
    #endif


    char time_buffer[64];
    std::strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", &tm_buf);

    formatter_->format(msg, formatted);

    // Store the formatted log message with its log level
    log_messages.push_back({fmt::to_string(formatted), std::string(time_buffer), fmt::to_string(msg.logger_name),
                            msg.level});

    // Limit the number of messages stored (e.g., 1000 messages)
    if (log_messages.size() > 1000) {
      log_messages.pop_front();
    }
  }


  // Optional: Flush function (you can leave this empty)
  void flush_() override {
  }

public:
  // Render the logs in the ImGui window
  void render() {
    ImGui::SetNextWindowPos(ImVec2(100.0f, 100.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(500, 400), ImGuiCond_FirstUseEver);
    ImGui::Begin("Log Window", &is_open, ImGuiWindowFlags_None);

    // Options menu
    if (ImGui::BeginPopup("Options")) {
      ImGui::Checkbox("Auto-scroll", &AutoScroll);
      ImGui::EndPopup();
    }

    // Main window buttons
    if (ImGui::Button("Options")) ImGui::OpenPopup("Options");
    ImGui::SameLine();
    bool clear = ImGui::Button("Clear");
    ImGui::SameLine();
    bool copy = ImGui::Button("Copy");
    ImGui::SameLine();
    Filter.Draw("Filter", -100.0f);

    ImGui::Separator();

    // Log scrolling window
    if (ImGui::BeginChild("scrolling", ImVec2(0, 0), true,
                          ImGuiWindowFlags_HorizontalScrollbar)) {
      if (clear) {
        log_messages.clear();
      }
      if (copy) {
        ImGui::LogToClipboard();
      }

      // Iterate through log messages and display with color on log level only
      for (const auto& entry : log_messages) {
        if (!Filter.PassFilter(entry.message.c_str())) {
          continue;
        }

        std::string display_time = "[" + entry.time + "]";
        ImGui::TextUnformatted(display_time.c_str());
        ImGui::SameLine();

        std::string display_logger_name = "[" + entry.logger_name + "]";
        ImGui::TextUnformatted(display_logger_name.c_str());
        ImGui::SameLine();

        const std::size_t pos = entry.message.find_last_of(
            "]", entry.message.size());
        std::string message_content = (pos != std::string::npos)
                                        ? entry.message.substr(pos + 1)
                                        : entry.message;

        // Split the log message into the log level and the rest of the message
        std::string level_tag = "[" + std::string(
                                    spdlog::level::to_string_view(entry.level).
                                    data()) + "] ";
        std::string log_message = entry.message.substr(level_tag.length());

        // Set color based on log level
        switch (entry.level) {
          case spdlog::level::info:
            ImGui::PushStyleColor(ImGuiCol_Text,
                                  ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
          // Green for info
            break;
          case spdlog::level::warn:
            ImGui::PushStyleColor(ImGuiCol_Text,
                                  ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
          // Yellow for warnings
            break;
          case spdlog::level::err:
          case spdlog::level::critical:
            ImGui::PushStyleColor(ImGuiCol_Text,
                                  ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
          // Red for errors/critical
            break;
          case spdlog::level::debug:
            ImGui::PushStyleColor(ImGuiCol_Text,
                                  ImVec4(0.0f, 0.5f, 1.0f, 1.0f));
          // Blue for debug
            break;
          default:
            ImGui::PushStyleColor(ImGuiCol_Text,
                                  ImGui::GetStyleColorVec4(ImGuiCol_Text));
          // Default color
            break;
        }

        // Display the log level in color
        ImGui::TextUnformatted(level_tag.c_str());

        // Revert to default color and display the rest of the message
        ImGui::PopStyleColor();
        ImGui::SameLine();
        ImGui::TextUnformatted(message_content.c_str());
      }

      // Auto-scroll to bottom if enabled and at the bottom already
      if (AutoScroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
        ImGui::SetScrollHereY(1.0f);
      }
    }
    ImGui::EndChild();

    ImGui::End();
  }
};

#endif //IMGUI_LOGGER_H
