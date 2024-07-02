
#include <salsa/salsa.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <CLI/CLI.hpp>

#include "plot/plot.h"
#include "testbed.h"
#include "user.h"
int main(int argc, char** argv) {
  CLI::App app{"Testbed for salsa simulation"};

  bool headless = false;
  bool verbose = false;
  bool no_plots = false;

  app.add_flag("--headless", headless, "Run in headless mode");
  app.add_flag("-v,--verbose", verbose, "Verbose output")->needs("--headless");
  app.add_flag("--no-plots", no_plots, "Do not plot the results");
  CLI11_PARSE(app, argc, argv);

  auto testbed_console = spdlog::stdout_color_mt("testbed_console");
  testbed_console->set_level(spdlog::level::info);
  // spdlog::register_logger(testbed_console);
  salsa::map::loadAll();

  if (headless) {
    // Run in headless mode
    testbed::user();
    if (!no_plots) {
      testbed::plot_drone_distances = true;
      testbed::plot_drone_heatmap = true;
      testbed::plot_drone_speed = true;
      testbed::plot_drone_trace = true;
      testbed::plot_targets_found = true;
    }
    testbed::run_headless(verbose);
  } else {
    testbed::user();
    testbed::run();
  }
  return 0;
}