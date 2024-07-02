#ifndef TESTBED_PLOT_H
#define TESTBED_PLOT_H

#include <Python.h>
#include <salsa/core/map.h>

#include <filesystem>
#include <string>

namespace testbed {

extern bool plot_targets_found;
extern bool plot_drone_speed;
extern bool plot_drone_trace;
extern bool plot_drone_heatmap;
extern bool plot_drone_distances;

void plot(std::string log_file_path);
}  // namespace testbed

#endif  // TESTBED_PLOT_H