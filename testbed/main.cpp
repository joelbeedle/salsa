
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
  std::string queue_path = "none";

  app.add_flag("--headless", headless, "Run in headless mode");
  app.add_flag("-v,--verbose", verbose, "Verbose output")->needs("--headless");
  app.add_flag("--no-plots", no_plots, "Do not plot the results")
      ->needs("--headless");
  app.add_option("-q,--queue", queue_path, "Path to the queue file")
      ->needs("--headless");
  CLI11_PARSE(app, argc, argv);

  auto testbed_console = spdlog::stdout_color_mt("testbed_console");
  testbed_console->set_level(spdlog::level::info);
  // spdlog::register_logger(testbed_console);
  salsa::map::loadAll();
  testbed::init_python();
  if (headless) {
    // Run in headless mode
    testbed::user();

    if (queue_path != "none") {
      std::cout << "Loading queue from " << queue_path << std::endl;
      if (salsa::TestQueue::load(queue_path)) {
        std::cout << "Queue loaded successfully" << std::endl;
      } else {
        std::cout << "Failed to load queue" << std::endl;
        return 1;
      }
    }
    testbed_console->set_level(spdlog::level::warn);
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
  testbed::finalize_python();
  return 0;
}