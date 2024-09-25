#ifndef RUN_SIM_H
#define RUN_SIM_H

#ifdef __EMSCRIPTEN__
#define GL_GLEXT_PROTOTYPES
#define EGL_EGLEXT_PROTOTYPES
#include <GLES3/gl3.h>  // OpenGL ES 3.0 header
#include <emscripten.h>
#include <emscripten/html5.h>
#endif
#include <GLFW/glfw3.h>
#include <salsa/salsa.h>

namespace testbed {
int run();
int run_headless(bool verbose, std::string queue_path);
};  // namespace testbed
#endif