
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <CLI/CLI.hpp>

#include "core/logger.h"
#include "core/simulation.h"
#include "plot/plot.h"
#include "testbed.h"
#include "user.h"
int main(int argc, char** argv) {
  CLI::App app{"Testbed for swarm simulation"};

  bool headless = false;

  app.add_flag("--headless", headless, "Run in headless mode");

  CLI11_PARSE(app, argc, argv);

  auto testbed_console = spdlog::stdout_color_mt("testbed_console");
  testbed_console->set_level(spdlog::level::info);
  // spdlog::register_logger(testbed_console);
  testbed::plot();
  if (headless) {
    // Run in headless mode
    std::cout << "Running in headless mode" << std::endl;
  }
  testbed::user();
  return 0;
}