
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
  bool verbose = false;

  app.add_flag("--headless", headless, "Run in headless mode");
  app.add_flag("-v,--verbose", verbose, "Verbose output")->needs("--headless");
  CLI11_PARSE(app, argc, argv);

  auto testbed_console = spdlog::stdout_color_mt("testbed_console");
  testbed_console->set_level(spdlog::level::info);
  // spdlog::register_logger(testbed_console);
  testbed::plot();
  swarm::map::loadAll();

  if (headless) {
    // Run in headless mode
    testbed::user();
    testbed::run_headless(verbose);
  } else {
    testbed::user();
    testbed::run();
  }
  return 0;
}