#include "plot.h"

namespace testbed {
std::filesystem::path exec_path = salsa::map::getExecutablePath();
bool plot_targets_found = false;
bool plot_drone_speed = false;
bool plot_drone_trace = false;
bool plot_drone_heatmap = false;
bool plot_drone_distances = false;

void init_python() {
  Py_Initialize();
  std::filesystem::path python_path =
      exec_path / ".." / ".." / "testbed" / "plot";
  std::string python_path_str = python_path.string();
  std::string complete_string = "sys.path.append(\"" + python_path_str + "\")";
  PyRun_SimpleString("import sys");
  PyRun_SimpleString(complete_string.c_str());
}

void finalize_python() { Py_Finalize(); }

bool call_function(const char *module_name, const char *function_name,
                   const char *argument = nullptr) {
  PyObject *pName, *pModule, *pFunc;
  PyObject *pArgs, *pValue = nullptr;

  pName = PyUnicode_DecodeFSDefault(module_name);
  pModule = PyImport_Import(pName);
  Py_DECREF(pName);

  if (pModule != NULL) {
    pFunc = PyObject_GetAttrString(pModule, function_name);
    if (pFunc && PyCallable_Check(pFunc)) {
      if (argument != nullptr) {
        pArgs = PyTuple_Pack(1, PyUnicode_DecodeFSDefault(argument));
        pValue = PyObject_CallObject(pFunc, pArgs);
        Py_DECREF(pArgs);
      } else {
        pValue = PyObject_CallObject(pFunc, NULL);
      }
      if (pValue != NULL) {
        Py_DECREF(pValue);
      } else {
        Py_DECREF(pFunc);
        Py_DECREF(pModule);
        PyErr_Print();
        return false;
      }
    } else {
      if (PyErr_Occurred()) PyErr_Print();
      Py_DECREF(pModule);
      return false;
    }
    Py_DECREF(pFunc);
    Py_DECREF(pModule);
  } else {
    PyErr_Print();
    return false;
  }
  return true;
}

void plot(std::string log_file_path) {
  std::filesystem::path results_dir =
      exec_path / ".." / ".." / "testbed" / "results";
  std::filesystem::path file_path = results_dir / (log_file_path);

  if (!call_function("py_plot", "set_file_name", log_file_path.c_str())) {
    return;
  }
  if (!call_function("py_plot", "set_output_path",
                     results_dir.string().c_str())) {
    return;
  }

  if (!call_function("py_plot", "get_sim_data", file_path.string().c_str())) {
    return;
  }

  // Create dataframe from log file
  if (!call_function("py_plot", "create_dataframe",
                     file_path.string().c_str())) {
    return;
  }
  if (testbed::plot_targets_found) {
    std::cout << "Plotting targets found" << std::endl;
    if (!call_function("py_plot", "plot_targets_found_wrapper")) {
      return;
    }
  }
  if (testbed::plot_drone_speed) {
    std::cout << "Plotting drone speed" << std::endl;
    if (!call_function("py_plot", "plot_speed_wrapper")) {
      return;
    }
  }
  if (testbed::plot_drone_distances) {
    std::cout << "Plotting drone distances" << std::endl;
    if (!call_function("py_plot", "plot_drone_distances_wrapper")) {
      return;
    }
  }
  if (testbed::plot_drone_trace) {
    std::cout << "Plotting drone trace" << std::endl;
    if (!call_function("py_plot", "plot_trace_wrapper")) {
      return;
    }
  }
  if (testbed::plot_drone_heatmap) {
    std::cout << "Plotting drone heatmap" << std::endl;
    if (!call_function("py_plot", "plot_heatmap_wrapper", "50")) {
      return;
    }
  }
  // Finalize Python
  return;
}

}  // namespace testbed