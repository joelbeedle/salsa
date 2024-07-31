#ifndef RUN_SIM_H
#define RUN_SIM_H

#include <salsa/salsa.h>

namespace testbed {
int run();
int run_headless(bool verbose, std::string queue_path);
};  // namespace testbed
#endif