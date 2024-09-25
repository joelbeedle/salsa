#ifndef TESTBED_PLOT_H
#define TESTBED_PLOT_H

#ifndef __EMSCRIPTEN__
#include <Python.h>
#endif
#include <salsa/core/map.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace testbed {

extern bool plot_targets_found;
extern bool plot_drone_speed;
extern bool plot_drone_trace;
extern bool plot_drone_heatmap;
extern bool plot_drone_distances;

void plot(std::string log_file_path);
void add_rtf_to_csv(std::string log_file_path, int num_drones, int num_targets,
                    double rtf);
void init_python();
void finalize_python();
}  // namespace testbed

#endif  // TESTBED_PLOT_H