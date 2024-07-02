#include "plot.h"

namespace testbed {

void init_python() {
  Py_Initialize();
  PyRun_SimpleString("import sys");
  PyRun_SimpleString(
      "sys.path.append('/Users/joelbeedle/swarm-sim/testbed/plot')");
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

void plot() {
  init_python();

  // Create dataframe from log file
  if (!call_function("py_plot", "create_dataframe",
                     "/Users/joelbeedle/swarm-sim/testbed/plot/"
                     "2024-07-02_19-00-16_Flocking.log")) {
    finalize_python();
    return;
  }

  if (!call_function("py_plot", "plot_targets_found_wrapper")) {
    finalize_python();
    return;
  }
  // Finalize Python
  finalize_python();
  return;
}

}  // namespace testbed