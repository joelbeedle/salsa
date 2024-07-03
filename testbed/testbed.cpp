// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "salsa/entity/drone_configuration.h"
#define _CRT_SECURE_NO_WARNINGS
#define IMGUI_DISABLE_OBSOLETE_FUNCTIONS 1

#include <stdio.h>

#include <algorithm>
#include <chrono>
#include <thread>

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "draw.h"
#include "imgui.h"
#include "settings.h"
#include "test.h"
#include "testbed.h"

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#if defined(_WIN32)
#include <crtdbg.h>
#endif
#include "plot/plot.h"

namespace testbed {

GLFWwindow *g_mainWindow = nullptr;
static int32 s_testSelection = 0;
static std::unique_ptr<Test> s_test = nullptr;
static Settings s_settings;
static bool s_rightMouseDown = false;
static b2Vec2 s_clickPointWS = b2Vec2_zero;
static float s_displayScale = 1.0f;
static bool start_menu = true;

void glfwErrorCallback(int error, const char *description) {
  fprintf(stderr, "GLFW error occured. Code: %d. Description: %s\n", error,
          description);
}

static inline bool CompareTests(const TestEntry &a, const TestEntry &b) {
  int result = strcmp(a.category, b.category);
  if (result == 0) {
    result = strcmp(a.name, b.name);
  }

  return result < 0;
}

static void RestartTest() {
  s_test = std::move(g_testEntries[s_settings.m_testIndex].instance());
}

static void CreateUI(GLFWwindow *window, const char *glslVersion = NULL) {
  // IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  bool success;
  success = ImGui_ImplGlfw_InitForOpenGL(window, false);
  if (success == false) {
    printf("ImGui_ImplGlfw_InitForOpenGL failed\n");
    assert(false);
  }

  success = ImGui_ImplOpenGL3_Init(glslVersion);
  if (success == false) {
    printf("ImGui_ImplOpenGL3_Init failed\n");
    assert(false);
  }

  // Search for font file
  const char *fontPath1 = "data/droid_sans.ttf";
  const char *fontPath2 = "../data/droid_sans.ttf";
  const char *fontPath = nullptr;
  FILE *file1 = fopen(fontPath1, "rb");
  FILE *file2 = fopen(fontPath2, "rb");
  if (file1) {
    fontPath = fontPath1;
    fclose(file1);
  }

  if (file2) {
    fontPath = fontPath2;
    fclose(file2);
  }

  if (fontPath) {
    ImGui::GetIO().Fonts->AddFontFromFileTTF(fontPath, 14.0f * s_displayScale);
  }
}

static void ResizeWindowCallback(GLFWwindow *, int width, int height) {
  g_camera.m_width = width;
  g_camera.m_height = height;
  s_settings.m_windowWidth = width;
  s_settings.m_windowHeight = height;
}

static void KeyCallback(GLFWwindow *window, int key, int scancode, int action,
                        int mods) {
  ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
  if (ImGui::GetIO().WantCaptureKeyboard) {
    return;
  }

  if (action == GLFW_PRESS) {
    switch (key) {
      case GLFW_KEY_ESCAPE:
        // Quit
        start_menu = !start_menu;
        s_settings.m_pause = !s_settings.m_pause;
        // glfwSetWindowShouldClose(g_mainWindow, GL_TRUE);
        break;

      case GLFW_KEY_LEFT:
        // Pan left
        if (mods == GLFW_MOD_CONTROL) {
          b2Vec2 newOrigin(2.0f, 0.0f);
          s_test->ShiftOrigin(newOrigin);
        } else {
          g_camera.m_center.x -= 0.5f;
        }
        break;

      case GLFW_KEY_RIGHT:
        // Pan right
        if (mods == GLFW_MOD_CONTROL) {
          b2Vec2 newOrigin(-2.0f, 0.0f);
          s_test->ShiftOrigin(newOrigin);
        } else {
          g_camera.m_center.x += 0.5f;
        }
        break;

      case GLFW_KEY_DOWN:
        // Pan down
        if (mods == GLFW_MOD_CONTROL) {
          b2Vec2 newOrigin(0.0f, 2.0f);
          s_test->ShiftOrigin(newOrigin);
        } else {
          g_camera.m_center.y -= 0.5f;
        }
        break;

      case GLFW_KEY_UP:
        // Pan up
        if (mods == GLFW_MOD_CONTROL) {
          b2Vec2 newOrigin(0.0f, -2.0f);
          s_test->ShiftOrigin(newOrigin);
        } else {
          g_camera.m_center.y += 0.5f;
        }
        break;

      case GLFW_KEY_HOME:
        g_camera.ResetView();
        break;

      case GLFW_KEY_Z:
        // Zoom out
        g_camera.m_zoom = b2Min(1.1f * g_camera.m_zoom, 20.0f);
        break;

      case GLFW_KEY_X:
        // Zoom in
        g_camera.m_zoom = b2Max(0.9f * g_camera.m_zoom, 0.02f);
        break;

      case GLFW_KEY_R:
        RestartTest();
        break;

      case GLFW_KEY_SPACE:
        // Launch a bomb.
        if (s_test) {
          s_test->LaunchBomb();
        }
        break;

      case GLFW_KEY_O:
        s_settings.m_singleStep = true;
        break;

      case GLFW_KEY_P:
        s_settings.m_pause = !s_settings.m_pause;
        break;

      case GLFW_KEY_LEFT_BRACKET:
        // Switch to previous test
        --s_testSelection;
        if (s_testSelection < 0) {
          s_testSelection = g_testCount - 1;
        }
        break;

      case GLFW_KEY_RIGHT_BRACKET:
        // Switch to next test
        ++s_testSelection;
        if (s_testSelection == g_testCount) {
          s_testSelection = 0;
        }
        break;

      case GLFW_KEY_TAB:
        g_debugDraw.m_showUI = !g_debugDraw.m_showUI;

      default:
        if (s_test) {
          s_test->Keyboard(key);
        }
    }
  } else if (action == GLFW_RELEASE) {
    s_test->KeyboardUp(key);
  }
  // else GLFW_REPEAT
}

static void CharCallback(GLFWwindow *window, unsigned int c) {
  ImGui_ImplGlfw_CharCallback(window, c);
}

static void MouseButtonCallback(GLFWwindow *window, int32 button, int32 action,
                                int32 mods) {
  ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);

  if (ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow)) {
    return;
  }

  double xd, yd;
  glfwGetCursorPos(g_mainWindow, &xd, &yd);
  b2Vec2 ps((float)xd, (float)yd);

  // Use the mouse to move things around.
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    //<##>
    // ps.Set(0, 0);
    b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);
    if (action == GLFW_PRESS) {
      if (mods == GLFW_MOD_SHIFT) {
        s_test->ShiftMouseDown(pw);
      } else {
        s_test->MouseDown(pw);
      }
    }

    if (action == GLFW_RELEASE) {
      s_test->MouseUp(pw);
    }
  } else if (button == GLFW_MOUSE_BUTTON_2) {
    if (action == GLFW_PRESS) {
      s_clickPointWS = g_camera.ConvertScreenToWorld(ps);
      s_rightMouseDown = true;
    }

    if (action == GLFW_RELEASE) {
      s_rightMouseDown = false;
    }
  }
  if (button == GLFW_MOUSE_BUTTON_RIGHT) {
    b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);

    if (action == GLFW_PRESS) {
      s_test->RightMouseDown(pw);
    }
  }
}

static void MouseMotionCallback(GLFWwindow *, double xd, double yd) {
  b2Vec2 ps((float)xd, (float)yd);

  b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);
  s_test->MouseMove(pw);

  if (s_rightMouseDown) {
    b2Vec2 diff = pw - s_clickPointWS;
    g_camera.m_center.x -= diff.x;
    g_camera.m_center.y -= diff.y;
    s_clickPointWS = g_camera.ConvertScreenToWorld(ps);
  }
}

static void ScrollCallback(GLFWwindow *window, double dx, double dy) {
  ImGui_ImplGlfw_ScrollCallback(window, dx, dy);
  if (ImGui::GetIO().WantCaptureMouse) {
    return;
  }

  if (dy > 0) {
    g_camera.m_zoom /= 1.1f;
  } else {
    g_camera.m_zoom *= 1.1f;
  }
}

static void UpdateUI() {
  float menuWidth = 180.0f * s_displayScale;
  ImGuiIO &io = ImGui::GetIO();
  ImGui::SetNextWindowPos(
      ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f),
      ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  ImGui::SetNextWindowSize(
      ImVec2(float(g_camera.m_width) / 4, float(g_camera.m_height) / 4));
  if (ImGui::BeginPopupModal("Menu", &start_menu,
                             ImGuiWindowFlags_NoMove |
                                 ImGuiWindowFlags_NoResize |
                                 ImGuiWindowFlags_NoCollapse)) {
    if (ImGui::Button("Start in Queue mode", ImVec2(-1, 0))) {
      start_menu = false;
      s_settings.m_pause = false;
      s_testSelection = 1;
    }

    if (ImGui::Button("Start in Sandbox mode", ImVec2(-1, 0))) {
      start_menu = false;
      s_settings.m_pause = false;
      s_testSelection = 2;
    }

    if (ImGui::Button("Map Creator", ImVec2(-1, 0))) {
      start_menu = false;
      s_settings.m_pause = false;
      s_testSelection = 0;
    }

    if (ImGui::Button("Exit", ImVec2(-1, 0))) {
      glfwSetWindowShouldClose(g_mainWindow, GL_TRUE);
    }
    ImGui::EndPopup();
  }

  if (!start_menu && g_debugDraw.m_showUI) {
    ImGui::SetNextWindowPos({g_camera.m_width - menuWidth - 10.0f, 10.0f});
    ImGui::SetNextWindowSize({menuWidth, g_camera.m_height - 20.0f});

    ImGui::Begin("Tools", &g_debugDraw.m_showUI,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoCollapse);

    if (ImGui::BeginTabBar("ControlTabs", ImGuiTabBarFlags_None)) {
      if (ImGui::BeginTabItem("Controls")) {
        ImGui::SliderInt("Vel Iters", &s_settings.m_velocityIterations, 0, 50);
        ImGui::SliderInt("Pos Iters", &s_settings.m_positionIterations, 0, 50);
        ImGui::SliderFloat("Hertz", &s_settings.m_hertz, 5.0f, 120.0f,
                           "%.0f hz");
        ImGui::SliderFloat("Simulation Speed", &s_settings.m_simulationSpeed,
                           0.1f, 20.0f, "%.1f x");

        ImGui::Separator();

        ImGui::Checkbox("Sleep", &s_settings.m_enableSleep);
        ImGui::Checkbox("Warm Starting", &s_settings.m_enableWarmStarting);
        ImGui::Checkbox("Time of Impact", &s_settings.m_enableContinuous);
        ImGui::Checkbox("Sub-Stepping", &s_settings.m_enableSubStepping);

        ImGui::Separator();

        ImGui::Checkbox("Shapes", &s_settings.m_drawShapes);
        ImGui::Checkbox("Joints", &s_settings.m_drawJoints);
        ImGui::Checkbox("AABBs", &s_settings.m_drawAABBs);
        ImGui::Checkbox("Contact Points", &s_settings.m_drawContactPoints);
        ImGui::Checkbox("Contact Normals", &s_settings.m_drawContactNormals);
        ImGui::Checkbox("Contact Impulses", &s_settings.m_drawContactImpulse);
        ImGui::Checkbox("Friction Impulses", &s_settings.m_drawFrictionImpulse);
        ImGui::Checkbox("Center of Masses", &s_settings.m_drawCOMs);
        ImGui::Checkbox("Statistics", &s_settings.m_drawStats);
        ImGui::Checkbox("Profile", &s_settings.m_drawProfile);

        ImVec2 button_sz = ImVec2(-1, 0);
        if (ImGui::Button("Pause (P)", button_sz)) {
          s_settings.m_pause = !s_settings.m_pause;
        }

        if (ImGui::Button("Single Step (O)", button_sz)) {
          s_settings.m_singleStep = !s_settings.m_singleStep;
        }

        if (ImGui::Button("Restart (R)", button_sz)) {
          RestartTest();
        }

        if (ImGui::Button("Quit", button_sz)) {
          glfwSetWindowShouldClose(g_mainWindow, GL_TRUE);
        }

        ImGui::EndTabItem();
      }

      ImGuiTreeNodeFlags leafNodeFlags =
          ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;
      leafNodeFlags |=
          ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen;

      ImGuiTreeNodeFlags nodeFlags =
          ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick;

      if (ImGui::BeginTabItem("Modes")) {
        int categoryIndex = 0;
        const char *category = g_testEntries[categoryIndex].category;
        int i = 0;
        while (i < g_testCount) {
          bool categorySelected =
              strcmp(category,
                     g_testEntries[s_settings.m_testIndex].category) == 0;
          ImGuiTreeNodeFlags nodeSelectionFlags =
              categorySelected ? ImGuiTreeNodeFlags_Selected : 0;
          bool nodeOpen =
              ImGui::TreeNodeEx(category, nodeFlags | nodeSelectionFlags);

          if (nodeOpen) {
            while (i < g_testCount &&
                   strcmp(category, g_testEntries[i].category) == 0) {
              ImGuiTreeNodeFlags selectionFlags = 0;
              if (s_settings.m_testIndex == i) {
                selectionFlags = ImGuiTreeNodeFlags_Selected;
              }
              ImGui::TreeNodeEx((void *)(intptr_t)i,
                                leafNodeFlags | selectionFlags, "%s",
                                g_testEntries[i].name);
              if (ImGui::IsItemClicked()) {
                s_testSelection = i;
              }
              ++i;
            }
            ImGui::TreePop();
          } else {
            while (i < g_testCount &&
                   strcmp(category, g_testEntries[i].category) == 0) {
              ++i;
            }
          }

          if (i < g_testCount) {
            category = g_testEntries[i].category;
            categoryIndex = i;
          }
        }
        ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
    }

    ImGui::End();

    s_test->UpdateUI();
  }
}

int run_headless(bool verbose) {
  s_settings.Load();

  s_settings.m_testIndex = b2Clamp(s_settings.m_testIndex, 0, g_testCount - 1);
  s_testSelection = s_settings.m_testIndex;
  s_test = std::move(g_testEntries[s_settings.m_testIndex].instance());
  b2World *world;
  salsa::TestConfig test = salsa::TestQueue::pop();
  salsa::TestQueue::push(test);

  bool init_testbed = true;
  salsa::Sim *sim;
  int count = 0;
  int original_size = salsa::TestQueue::size();
  while (salsa::TestQueue::size() > 0) {
    try {
      auto test = salsa::TestQueue::pop();
      auto temp_sim = new salsa::Sim(test);
      if (temp_sim == nullptr) {
        return false;
      }
      if (init_testbed) {
        init_testbed = false;
        sim = temp_sim;
        sim->setCurrentBehaviour(sim->current_behaviour_name());
        world = sim->getWorld();
      } else {
        auto old_sim = sim;
        sim = temp_sim;
        delete old_sim;
        sim->setCurrentBehaviour(sim->current_behaviour_name());
        world = sim->getWorld();
      }
      std::cout << "(" << count << "/" << original_size << ")"
                << " Running test: " << test.behaviour_name << std::endl;
      std::cout << "Drones: " << test.num_drones
                << " Targets: " << test.num_targets << std::endl;
      bool first_run = true;
      auto totalStart = std::chrono::high_resolution_clock::now();
      auto updateStart = std::chrono::steady_clock::now();

      while (sim->current_time() < test.time_limit) {
        world->Step(1 / 60.0f, 8, 3);
        sim->update();
        sim->current_time() += (1.0f / 60.0f) * 1.0f;

        // Display Progress
        auto now = std::chrono::steady_clock::now();
        auto elapsed =
            std::chrono::duration_cast<std::chrono::seconds>(now - updateStart)
                .count();
        if (verbose && (elapsed >= 3 || first_run ||
                        sim->current_time() >= test.time_limit)) {
          updateStart = now;
          const int cTotalLength = 20;
          float lProgress = sim->current_time() / test.time_limit;
          std::cout << "\r" << std::string(cTotalLength + 10, ' ') << "\r";
          std::cout << "[" << std::string(int(cTotalLength * lProgress), '#')
                    << std::string(int(cTotalLength * (1 - lProgress)), '-')
                    << "] " << std::setprecision(3) << 100 * lProgress << "%"
                    << std::flush;
          std::cout << "\t" << sim->current_time() << "s / " << test.time_limit
                    << "s" << std::endl;
          first_run = false;
        }
      }

      auto totalEnd = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> totalDuration =
          totalEnd - totalStart;
      double time_limit_milliseconds =
          static_cast<double>(test.time_limit * 1000.0);
      double time_taken_milliseconds = totalDuration.count();
      std::cout << "Time taken: " << time_taken_milliseconds << "ms"
                << " Time limit: " << time_limit_milliseconds << "ms"
                << std::endl;
      // Calculate the ratio of time taken to the time limit
      double ratio = time_taken_milliseconds / time_limit_milliseconds;

      std::cout << std::endl;
      std::cout << "Finished test " << test.behaviour_name << " ";
      std::cout << "(RTF: " << ratio << ")" << std::endl;
      testbed::plot(sim->getCurrentLogFile());
    } catch (const std::exception &e) {
      spdlog::error("Error: {}", e.what());
      std::cerr << e.what() << std::endl;
    }
  }

  s_test = nullptr;

  s_settings.Save();

  return 0;
}

//
int run() {
#if defined(_WIN32)
  // Enable memory-leak reports
  _CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
#endif

  char buffer[128];

  s_settings.Load();

  glfwSetErrorCallback(glfwErrorCallback);

  g_camera.m_width = s_settings.m_windowWidth;
  g_camera.m_height = s_settings.m_windowHeight;

  if (glfwInit() == 0) {
    fprintf(stderr, "Failed to initialize GLFW\n");
    return -1;
  }

#if __APPLE__
  const char *glslVersion = "#version 150";
#else
  const char *glslVersion = NULL;
#endif

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  sprintf(buffer, "Box2D Testbed Version %d.%d.%d", b2_version.major,
          b2_version.minor, b2_version.revision);

  bool fullscreen = false;
  if (fullscreen) {
    g_mainWindow =
        glfwCreateWindow(1920, 1080, buffer, glfwGetPrimaryMonitor(), NULL);
  } else {
    g_mainWindow = glfwCreateWindow(g_camera.m_width, g_camera.m_height, buffer,
                                    NULL, NULL);
  }

  if (g_mainWindow == NULL) {
    fprintf(stderr, "Failed to open GLFW g_mainWindow.\n");
    glfwTerminate();
    return -1;
  }

  glfwGetWindowContentScale(g_mainWindow, &s_displayScale, &s_displayScale);

  glfwMakeContextCurrent(g_mainWindow);

  // Load OpenGL functions using glad
  int version = gladLoadGL(glfwGetProcAddress);
  printf("GL %d.%d\n", GLAD_VERSION_MAJOR(version),
         GLAD_VERSION_MINOR(version));
  printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION),
         glGetString(GL_SHADING_LANGUAGE_VERSION));

  glfwSetWindowSizeCallback(g_mainWindow, ResizeWindowCallback);
  glfwSetKeyCallback(g_mainWindow, KeyCallback);
  glfwSetCharCallback(g_mainWindow, CharCallback);
  glfwSetMouseButtonCallback(g_mainWindow, MouseButtonCallback);
  glfwSetCursorPosCallback(g_mainWindow, MouseMotionCallback);
  glfwSetScrollCallback(g_mainWindow, ScrollCallback);

  g_debugDraw.Create();

  CreateUI(g_mainWindow, glslVersion);

  s_settings.m_testIndex = b2Clamp(s_settings.m_testIndex, 0, g_testCount - 1);
  s_testSelection = s_settings.m_testIndex;
  s_test = std::move(g_testEntries[s_settings.m_testIndex].instance());

  // Control the frame rate. One draw per monitor refresh.
  // glfwSwapInterval(1);

  glClearColor(0.2f, 0.2f, 0.2f, 1.0f);

  std::chrono::duration<double> frameTime(0.0);
  std::chrono::duration<double> sleepAdjust(0.0);

  while (!glfwWindowShouldClose(g_mainWindow)) {
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    glfwGetWindowSize(g_mainWindow, &g_camera.m_width, &g_camera.m_height);

    int bufferWidth, bufferHeight;
    glfwGetFramebufferSize(g_mainWindow, &bufferWidth, &bufferHeight);
    glViewport(0, 0, bufferWidth, bufferHeight);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();

    ImGui::NewFrame();

    if (g_debugDraw.m_showUI) {
      ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
      ImGui::SetNextWindowSize(
          ImVec2(float(g_camera.m_width), float(g_camera.m_height)));
      ImGui::SetNextWindowBgAlpha(0.0f);
      ImGui::Begin("Overlay", nullptr,
                   ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs |
                       ImGuiWindowFlags_AlwaysAutoResize |
                       ImGuiWindowFlags_NoScrollbar);
      ImGui::End();

      const TestEntry &entry = g_testEntries[s_settings.m_testIndex];
      sprintf(buffer, "%s : %s", entry.category, entry.name);
      s_test->DrawTitle(buffer);
    }

    s_test->Step(s_settings);

    if (start_menu) {
      ImGui::OpenPopup("Menu");
      s_settings.m_pause = true;
    }
    UpdateUI();

    // ImGui::ShowDemoWindow();

    if (g_debugDraw.m_showUI) {
      sprintf(buffer, "%.1f ms", 1000.0 * frameTime.count());
      g_debugDraw.DrawString(5, g_camera.m_height - 20, buffer);
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(g_mainWindow);

    if (s_testSelection != s_settings.m_testIndex) {
      s_settings.m_testIndex = s_testSelection;
      s_test = std::move(g_testEntries[s_settings.m_testIndex].instance());
      g_camera.ResetView();
    }

    glfwPollEvents();

    // Throttle to cap at 60Hz. This adaptive using a sleep adjustment. This
    // could be improved by using mm_pause or equivalent for the last
    // millisecond.
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> target(1.0 / 60.0);
    std::chrono::duration<double> timeUsed = t2 - t1;
    std::chrono::duration<double> sleepTime = target - timeUsed + sleepAdjust;
    if (sleepTime > std::chrono::duration<double>(0)) {
      std::this_thread::sleep_for(sleepTime);
    }

    std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
    frameTime = t3 - t1;

    // Compute the sleep adjustment using a low pass filter
    sleepAdjust = 0.9 * sleepAdjust + 0.1 * (target - frameTime);
  }

  s_test = nullptr;

  g_debugDraw.Destroy();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  glfwTerminate();

  s_settings.Save();

  return 0;
}
}  // namespace testbed
