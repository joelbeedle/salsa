#include <box2d/box2d.h>
#include <salsa/core/map.h>

#include <filesystem>
#include <fstream>
#include <iostream>

#include "box2d/b2_body.h"
#include "box2d/b2_math.h"
#include "imgui.h"
#include "nlohmann/json.hpp"
#include "settings.h"
#include "test.h"
using namespace salsa;

class myQueryCallback : public b2QueryCallback {
 public:
  std::vector<b2Body *> bodies;
  bool ReportFixture(b2Fixture *fixture) override {
    b2Body *body = fixture->GetBody();
    bodies.push_back(body);
    // Continue the query.
    return true;
  }
};
class MapCreator : public Test {
 private:
  enum DrawMode {
    DRAW_LINE,
    DRAW_POLYGON,
    DRAW_HOLLOW_POLYGON,
    DRAW_CIRCLE,
    DRAW_NONE,
    DRAW_DRONE_SPAWN,
  };
  char map_name[128];
  DrawMode current_mode = DRAW_NONE;
  bool line_drawing = false;
  b2Vec2 line_spawn_point;
  std::vector<b2Vec2> points;
  float circle_radius = 0.0f;

  b2Vec2 drone_spawn_point;

  bool draw_boundary = false;
  std::vector<b2Body *> boundary_bodies;
  float boundary_side_length = 0.0f;

  bool body_popup = false;
  b2Body *selected_body = nullptr;

  bool save_map = false;
  bool open_map = false;
  bool new_map = false;
  bool saved_map_as = false;
  bool save_map_as = false;
  bool pause = false;

 public:
  MapCreator() { g_debugDraw.SetFlags(b2Draw::e_shapeBit); }
  void DeleteBoundary() {
    for (b2Body *body : boundary_bodies) {
      m_world->DestroyBody(body);
    }
    boundary_bodies.clear();
  }

  void CreateBoundary() {
    b2BodyDef groundBodyDef;
    groundBodyDef.position.Set(0.0f, 0.0f);

    b2Body *groundBody = m_world->CreateBody(&groundBodyDef);

    b2EdgeShape groundBox;

    // bottom
    groundBox.SetTwoSided(b2Vec2(0.0f, 0.0f),
                          b2Vec2(boundary_side_length, 0.0f));
    groundBody->CreateFixture(&groundBox, 0.0f);

    // top
    groundBox.SetTwoSided(b2Vec2(0.0f, boundary_side_length),
                          b2Vec2(boundary_side_length, boundary_side_length));
    groundBody->CreateFixture(&groundBox, 0.0f);

    // left
    groundBox.SetTwoSided(b2Vec2(0.0f, 0.0f),
                          b2Vec2(0.0f, boundary_side_length));
    groundBody->CreateFixture(&groundBox, 0.0f);

    // right
    groundBox.SetTwoSided(b2Vec2(boundary_side_length, 0.0f),
                          b2Vec2(boundary_side_length, boundary_side_length));
    groundBody->CreateFixture(&groundBox, 0.0f);

    boundary_bodies.push_back(groundBody);
  }
  void CreateHollowPolygon(const std::vector<b2Vec2> &vertices) {
    if (vertices.size() < 3)
      return;  // Need at least two points to start drawing edges
    for (size_t i = 0; i < vertices.size() - 1; i++) {
      CreateEdge(vertices[i], vertices[i + 1]);
    }
    // Optionally close the polygon
    CreateEdge(vertices.back(), vertices.front());
    line_drawing = false;
  }

  void CreateEdge(const b2Vec2 &start, const b2Vec2 &end) {
    b2BodyDef bd;
    bd.type = b2_staticBody;
    b2Body *body = m_world->CreateBody(&bd);

    b2EdgeShape shape;
    shape.SetTwoSided(start, end);
    body->CreateFixture(&shape, 0.0f);
  }

  void CreatePolygon(const std::vector<b2Vec2> &vertices) {
    if (vertices.size() < 3) {
      return;
    }
    b2BodyDef bd;
    bd.type = b2_staticBody;
    b2Body *body = m_world->CreateBody(&bd);

    b2PolygonShape shape;
    shape.Set(&vertices[0], (int32)vertices.size());

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &shape;
    fixtureDef.density = 1.0f;  // Example density

    body->CreateFixture(&fixtureDef);

    line_drawing = false;
  }

  void CreateCircle(const b2Vec2 &center, float radius) {
    b2BodyDef bodyDef;
    bodyDef.type = b2_staticBody;
    bodyDef.position =
        center;  // Set the body's position to the center parameter

    // Create the body using the definition in the world
    b2Body *body = m_world->CreateBody(&bodyDef);

    // Define a circle shape
    b2CircleShape circleShape;
    circleShape.m_p.Set(0.0f,
                        0.0f);  // Position the shape at the body's local origin
    circleShape.m_radius = radius;  // Set the radius of the circle

    // Create a fixture with the shape. Density is set to 0.0f because it's a
    // static body
    body->CreateFixture(&circleShape, 0.0f);
    line_drawing = false;
  }

  void BeginLine(const b2Vec2 &worldPt) {
    line_spawn_point = worldPt;
    line_drawing = true;
  }

  void CompleteLine(const b2Vec2 &worldPt) {
    if (!line_drawing) {
      return;
    }
    std::cout << "CompleteLine\n";

    b2Vec2 diff = worldPt - line_spawn_point;
    float length = diff.Length();
    if (length < 0.1f) {
      return;
    }
    b2BodyDef bd;
    b2Body *body = m_world->CreateBody(&bd);
    b2EdgeShape shape;
    shape.SetTwoSided(line_spawn_point, worldPt);
    body->CreateFixture(&shape, 0.0f);
    line_drawing = false;
  }

  void MouseDown(const b2Vec2 &p) override {
    m_mouseWorld = p;

    if (m_mouseJoint != NULL) {
      return;
    }

    if (current_mode == DRAW_POLYGON || current_mode == DRAW_LINE ||
        current_mode == DRAW_HOLLOW_POLYGON) {
      BeginLine(p);
    }

    if (current_mode == DRAW_CIRCLE) {
      line_spawn_point = p;
      line_drawing = true;
    }

    if (current_mode == DRAW_DRONE_SPAWN) {
      line_drawing = false;
      drone_spawn_point = p;
    }
  }

  void RightMouseDown(const b2Vec2 &p) override {
    m_mouseWorld = p;
    if (m_mouseJoint != NULL) {
      return;
    }
    b2AABB aabb;
    b2Vec2 d;

    d.Set(0.001f, 0.001f);
    aabb.lowerBound = p - d;
    aabb.upperBound = p - d;

    // Query the world for overlapping shapes.
    myQueryCallback callback;
    m_world->QueryAABB(&callback, aabb);
    for (auto body : callback.bodies) {
      body_popup = true;
      selected_body = body;
    }
  }

  void MouseUp(const b2Vec2 &p) override {
    if (m_mouseJoint) {
      m_world->DestroyJoint(m_mouseJoint);
      m_mouseJoint = NULL;
    }

    if (current_mode == DRAW_POLYGON && line_drawing) {
      if (points.size() > 1 &&
          (p - points.front()).Length() < g_camera.m_zoom * 1.0f) {
        CreatePolygon(points);
        points.clear();  // Clear the points to start a new shape
      } else {
        points.push_back(p);
      }
    } else if (current_mode == DRAW_HOLLOW_POLYGON) {
      if (points.size() > 1 &&
          (p - points.front()).Length() < g_camera.m_zoom * 1.0f) {
        CreateHollowPolygon(points);
        points.clear();
      } else {
        // Create an edge to the current point
        points.push_back(p);
      }
    } else if (current_mode == DRAW_LINE && line_drawing) {
      CompleteLine(p);
    } else if (current_mode == DRAW_CIRCLE && line_drawing) {
      circle_radius = (p - line_spawn_point).Length();
      CreateCircle(line_spawn_point, circle_radius);
    } else if (current_mode == DRAW_DRONE_SPAWN) {
    }
  }

  void MouseMove(const b2Vec2 &p) override {
    if (line_drawing) {
      m_mouseWorld = p;  // Update current mouse position for drawing
    }
  }

  void Keyboard(int key) override {
    switch (key) {
      case GLFW_KEY_Q:
        if (line_drawing) {
          line_drawing = false;
        } else {
          current_mode = DRAW_NONE;
          points.clear();
        }
        break;
      default:
        break;
    }
  }

  void SaveMap() {
    map::Map new_map = {map_name, boundary_side_length, boundary_side_length,
                        drone_spawn_point, m_world};
    map::save(new_map);
    pause = false;
  }

  void LoadMap(const char *new_map_name) {
    map::Map map = map::load(new_map_name);
    b2World *world = map.world;
    std::copy(new_map_name, new_map_name + 128, map_name);

    m_world->SetDebugDraw(nullptr);     // Detach from the old world
    world->SetDebugDraw(&g_debugDraw);  // Attach to the new world

    delete m_world;
    m_world = world;
    PrintBodiesAndFixtures(m_world);
    saved_map_as = true;
    pause = false;
  }

  void PrintBodiesAndFixtures(b2World *world) {
    for (b2Body *body = world->GetBodyList(); body; body = body->GetNext()) {
      std::cout << "Body: " << body << std::endl;
      for (b2Fixture *fixture = body->GetFixtureList(); fixture;
           fixture = fixture->GetNext()) {
        std::cout << "Fixture: " << fixture << std::endl;
        std::cout << "Shape Type: " << fixture->GetShape()->GetType()
                  << std::endl;
      }
    }
  }

  static std::unique_ptr<Test> Create() {
    return std::make_unique<MapCreator>();
  }
  void Step(Settings &settings) override {
    Test::Step(settings);
    settings.m_pause = pause;
    m_world->DebugDraw();
    // g_debugDraw.DrawString(drone_spawn_point, "Drone Spawn Point");
    g_debugDraw.DrawCircle(drone_spawn_point, 5.0f, b2Color(0.0f, 1.0f, 0.0f));
    if (line_drawing) {
      b2Color c(0.9f, 0.9f, 0.9f);
      switch (current_mode) {
        case DRAW_LINE:
          g_debugDraw.DrawSegment(line_spawn_point, m_mouseWorld, c);
          break;
        case DRAW_HOLLOW_POLYGON:
          g_debugDraw.DrawSegment(line_spawn_point, m_mouseWorld, c);
          break;
        case DRAW_POLYGON:
          g_debugDraw.DrawSegment(line_spawn_point, m_mouseWorld, c);
          break;
        case DRAW_CIRCLE:
          g_debugDraw.DrawSegment(line_spawn_point, m_mouseWorld, c);

          g_debugDraw.DrawCircle(line_spawn_point,
                                 (m_mouseWorld - line_spawn_point).Length(), c);
          break;
        case DRAW_DRONE_SPAWN:
          g_debugDraw.DrawSegment(line_spawn_point, m_mouseWorld, c);
          g_debugDraw.DrawCircle(line_spawn_point,
                                 (m_mouseWorld - line_spawn_point).Length(), c);
          break;
        default:
          break;
      }
    }

    switch (current_mode) {
      case DRAW_LINE:
        g_debugDraw.DrawString(5, m_textLine, "Draw Mode: Line");
        break;
      case DRAW_HOLLOW_POLYGON:
        g_debugDraw.DrawString(5, m_textLine, "Draw Mode: Hollow Polygon");
        break;
      case DRAW_POLYGON:
        g_debugDraw.DrawString(5, m_textLine, "Draw Mode: Polygon");
        break;
      case DRAW_CIRCLE:
        g_debugDraw.DrawString(5, m_textLine, "Draw Mode: Circle");
        break;
      case DRAW_DRONE_SPAWN:
        g_debugDraw.DrawString(5, m_textLine, "Draw Mode: Drone Spawn");
        break;
      default:
        g_debugDraw.DrawString(5, m_textLine, "Draw Mode: None");
        break;
    }
    // Increase the text line by one
    m_textLine += m_textIncrement;
    g_debugDraw.DrawString(5, m_textLine, "Press Q to clear drawing");
    m_textLine += m_textIncrement;
    g_debugDraw.DrawString(5, m_textLine, "Map: %s", map_name);
    m_textLine += m_textIncrement;
    if (current_mode == DRAW_POLYGON && points.size() > 1) {
      for (size_t i = 0; i < points.size() - 1; ++i) {
        g_debugDraw.DrawSegment(points[i], points[i + 1],
                                b2Color(0.9f, 0.9f, 0.9f));
      }
    }
    if (current_mode == DRAW_HOLLOW_POLYGON && points.size() > 1) {
      for (size_t i = 0; i < points.size() - 1; ++i) {
        g_debugDraw.DrawSegment(points[i], points[i + 1],
                                b2Color(0.9f, 0.9f, 0.9f));
      }
    }
  }
  void ShowMenu() {
    ImGui::MenuItem("(demo menu)", NULL, false, false);
    if (ImGui::MenuItem("New")) {
      new_map = true;
    }
    if (ImGui::MenuItem("Open", "Ctrl+O")) {
      open_map = true;
    }
    if (ImGui::MenuItem("Save", "Ctrl+S")) {
      if (saved_map_as) {
        SaveMap();
      } else {
        save_map_as = true;
      }
    }
    if (ImGui::MenuItem("Save As..")) {
      save_map_as = true;
    }
  }
  void UpdateUI() override {
    // Map Creator Controls Section
    ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
    ImGui::SetNextWindowSize(ImVec2(285.0f, 285.0f));
    ImGui::Begin("Map Creator Controls", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_MenuBar);
    ImGui::Text("Camera Zoom: %f", g_camera.m_zoom);

    // Drawing Controls
    static int selected = -1;
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    if (ImGui::TreeNode("Drawing Controls")) {
      if (ImGui::Selectable("Line", selected == 0)) selected = 0;
      if (ImGui::Selectable("Polygon", selected == 1)) selected = 1;
      if (ImGui::Selectable("Hollow Polygon", selected == 2)) selected = 2;
      if (ImGui::Selectable("Circle", selected == 3)) selected = 3;
      if (ImGui::Selectable("None", selected == 4)) selected = 4;
      ImGui::TreePop();
    }
    ImGui::Separator();
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    if (ImGui::TreeNode("Drone Spawn Controls")) {
      if (ImGui::Button("Drone Spawn")) {
        points.clear();
        line_drawing = false;
        selected = 5;
      }
      ImGui::TreePop();
    }

    current_mode = (DrawMode)selected;

    ImGui::Separator();
    bool to_save = ImGui::Button("Save Map");
    if (ImGui::BeginMenuBar()) {
      if (ImGui::BeginMenu("File")) {
        ShowMenu();
        ImGui::EndMenu();
      }
      if (ImGui::BeginMenu("Edit")) {
        ImGui::MenuItem("Dummy");
        ImGui::EndMenu();
      }
      ImGui::EndMenuBar();
    }
    if (save_map_as) {
      pause = true;
      ImGui::OpenPopup("Save As");
      save_map_as = false;
    }
    ImGuiIO &io = ImGui::GetIO();
    ImGui::SetNextWindowPos(
        ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f),
        ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

    if (ImGui::BeginPopupModal("Save As", NULL,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
      static char str1[128] = "";
      if (ImGui::IsWindowFocused() && !ImGui::IsAnyItemActive() &&
          !ImGui::IsMouseClicked(0))
        ImGui::SetKeyboardFocusHere(0);
      ImGui::InputText("Map Name", str1, IM_ARRAYSIZE(str1));
      ImGui::Separator();

      if (ImGui::Button("Save", ImVec2(120, 0))) {
        std::copy(str1, str1 + 128, map_name);
        saved_map_as = true;
        SaveMap();
        ImGui::CloseCurrentPopup();
      }
      ImGui::SetItemDefaultFocus();
      ImGui::SameLine();
      if (ImGui::Button("Cancel", ImVec2(120, 0))) {
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }

    if (open_map) {
      pause = true;
      ImGui::OpenPopup("Open Map");
      open_map = false;
    }

    if (ImGui::BeginPopupModal("Open Map", NULL,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
      static char str1[128] = "";
      if (ImGui::IsAnyItemFocused() && !ImGui::IsAnyItemActive() &&
          !ImGui::IsMouseClicked(0))
        ImGui::SetKeyboardFocusHere(0);
      ImGui::InputText("Map Name", str1, IM_ARRAYSIZE(str1));
      ImGui::Separator();
      if (ImGui::Button("Open", ImVec2(120, 0))) {
        LoadMap(str1);
        ImGui::CloseCurrentPopup();
      }
      ImGui::SetItemDefaultFocus();
      ImGui::SameLine();
      if (ImGui::Button("Cancel", ImVec2(120, 0))) {
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }

    // Boundary Setting Section
    ImGui::End();
    ImGui::SetNextWindowPos(ImVec2(10.0f, 400.0f));
    ImGui::SetNextWindowSize(ImVec2(285.0f, 285.0f));
    ImGui::Begin("Boundary Settings", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
    ImGui::Checkbox("Draw Boundary", &draw_boundary);
    bool changed = false;
    if (draw_boundary) {
      ImGui::Text("Boundary Length");
      changed = ImGui::SliderFloat("Length", &boundary_side_length, 0.0f,
                                   4000.0f, "%.1fm");
      if (changed) {
        DeleteBoundary();
        CreateBoundary();
      }
    }

    if (body_popup) {
      ImGui::OpenPopup("Body Popup");
      body_popup = false;
    }

    if (ImGui::BeginPopup("Body Popup")) {
      if (selected_body) {
        ImGui::Text("BodyID: %p", selected_body);
        ImGui::Text("BodyType: %d", (b2BodyType)selected_body->GetType());
        ImGui::Text("Position: (%f, %f)", selected_body->GetPosition().x,
                    selected_body->GetPosition().y);
        if (ImGui::Button("Delete")) {
          m_world->DestroyBody(selected_body);
          selected_body = nullptr;
          ImGui::CloseCurrentPopup();
        }
      }
      ImGui::EndPopup();
    }
    ImGui::End();
  }
};
static int testIndex =
    RegisterTest("Map Creator", "Map Creator", MapCreator::Create);
