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

#define _CRT_SECURE_NO_WARNINGS
#include "settings.h"

#include <stdio.h>

#include <fstream>
#include <iostream>
#include <stdexcept>

#include "nlohmann/json.hpp"
static const char *fileName = "settings.ini";

// Load a file. You must free the character array.
static bool sReadFile(char *&data, int &size, const char *filename) {
  FILE *file = fopen(filename, "rb");
  if (file == nullptr) {
    return false;
  }

  fseek(file, 0, SEEK_END);
  size = ftell(file);
  fseek(file, 0, SEEK_SET);

  if (size == 0) {
    return false;
  }

  data = (char *)malloc(size + 1);
  fread(data, size, 1, file);
  fclose(file);
  data[size] = 0;

  return true;
}

void Settings::Load() {
  std::ifstream i(fileName);
  if (!i.is_open()) {
    return;
  }

  nlohmann::json j;
  try {
    i >> j;
  } catch (nlohmann::json::parse_error &e) {
    return;  // Optional: handle or log the error
  }

  m_testIndex = j.value("testIndex", m_testIndex);
  m_windowWidth = j.value("windowWidth", m_windowWidth);
  m_windowHeight = j.value("windowHeight", m_windowHeight);
  m_hertz = j.value("hertz", m_hertz);
  m_velocityIterations = j.value("velocityIterations", m_velocityIterations);
  m_positionIterations = j.value("positionIterations", m_positionIterations);
  m_drawShapes = j.value("drawShapes", m_drawShapes);
  m_drawJoints = j.value("drawJoints", m_drawJoints);
  m_drawAABBs = j.value("drawAABBs", m_drawAABBs);
  m_drawContactPoints = j.value("drawContactPoints", m_drawContactPoints);
  m_drawContactNormals = j.value("drawContactNormals", m_drawContactNormals);
  m_drawContactImpulse = j.value("drawContactImpulse", m_drawContactImpulse);
  m_drawFrictionImpulse = j.value("drawFrictionImpulse", m_drawFrictionImpulse);
  m_drawCOMs = j.value("drawCOMs", m_drawCOMs);
  m_drawStats = j.value("drawStats", m_drawStats);
  m_drawProfile = j.value("drawProfile", m_drawProfile);
  m_enableWarmStarting = j.value("enableWarmStarting", m_enableWarmStarting);
  m_enableContinuous = j.value("enableContinuous", m_enableContinuous);
  m_enableSubStepping = j.value("enableSubStepping", m_enableSubStepping);
  m_enableSleep = j.value("enableSleep", m_enableSleep);
  m_simulationSpeed = j.value("simulationSpeed", m_simulationSpeed);
}

void Settings::Save() {
  nlohmann::json j;
  j["testIndex"] = m_testIndex;
  j["windowWidth"] = m_windowWidth;
  j["windowHeight"] = m_windowHeight;
  j["hertz"] = m_hertz;
  j["velocityIterations"] = m_velocityIterations;
  j["positionIterations"] = m_positionIterations;
  j["drawShapes"] = m_drawShapes;
  j["drawJoints"] = m_drawJoints;
  j["drawAABBs"] = m_drawAABBs;
  j["drawContactPoints"] = m_drawContactPoints;
  j["drawContactNormals"] = m_drawContactNormals;
  j["drawContactImpulse"] = m_drawContactImpulse;
  j["drawFrictionImpulse"] = m_drawFrictionImpulse;
  j["drawCOMs"] = m_drawCOMs;
  j["drawStats"] = m_drawStats;
  j["drawProfile"] = m_drawProfile;
  j["enableWarmStarting"] = m_enableWarmStarting;
  j["enableContinuous"] = m_enableContinuous;
  j["enableSubStepping"] = m_enableSubStepping;
  j["enableSleep"] = m_enableSleep;
  j["simulationSpeed"] = m_simulationSpeed;

  std::ofstream o(fileName);
  o << j.dump(4);  // serialize with pretty printing, use a tab width of 4
}
