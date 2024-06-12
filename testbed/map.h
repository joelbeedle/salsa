#include <box2d/box2d.h>

#include <iostream>

#include "box2d/b2_math.h"
#include "imgui/imgui.h"
#include "test.h"

class MapCreator : public Test {
 private:
  enum DrawMode {
    DRAW_NONE,
    DRAW_DRONE_SPAWN,
    DRAW_LINE,
    DRAW_POLYGON,
    DRAW_HOLLOW_POLYGON,
    DRAW_CIRCLE
  };
  DrawMode current_mode = DRAW_NONE;
  bool line_drawing = false;
  b2Vec2 line_spawn_point;
  std::vector<b2Vec2> points;
  float circle_radius = 0.0f;

  b2Vec2 drone_spawn_point;

 public:
  MapCreator() {}
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

    body->CreateFixture(&shape, 0.0f);
    line_drawing = false;
  }

  void CreateCircle(const b2Vec2 &center, float radius) {
    b2BodyDef bd;
    bd.type = b2_staticBody;
    b2Body *body = m_world->CreateBody(&bd);

    b2CircleShape shape;
    shape.m_p = center;
    shape.m_radius = radius;
    body->CreateFixture(&shape, 0.0f);
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

  void MouseUp(const b2Vec2 &p) override {
    if (m_mouseJoint) {
      m_world->DestroyJoint(m_mouseJoint);
      m_mouseJoint = NULL;
    }

    if (current_mode == DRAW_POLYGON) {
      if (points.size() > 1 && (p - points.front()).Length() < 5.0f) {
        CreatePolygon(points);
        points.clear();  // Clear the points to start a new shape
      } else {
        points.push_back(p);
      }
    } else if (current_mode == DRAW_HOLLOW_POLYGON) {
      if (points.size() > 1 && (p - points.front()).Length() < 5.0f) {
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

  static Test *Create() { return new MapCreator(); }
  void Step(Settings &settings) override {
    Test::Step(settings);
    m_world->DebugDraw();
    g_debugDraw.DrawText(10, 10, "Drone Spawn Point");
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

    if (current_mode == DRAW_POLYGON && points.size() > 1) {
      for (size_t i = 0; i < points.size() - 1; ++i) {
        g_debugDraw.DrawSegment(points[i], points[i + 1],
                                b2Color(0.9f, 0.9f, 0.9f));
      }
      if (line_drawing) {
        g_debugDraw.DrawSegment(points.back(), m_mouseWorld,
                                b2Color(0.9f, 0.9f, 0.9f));
      }
    }
    if (current_mode == DRAW_HOLLOW_POLYGON && points.size() > 1) {
      for (size_t i = 0; i < points.size() - 1; ++i) {
        g_debugDraw.DrawSegment(points[i], points[i + 1],
                                b2Color(0.9f, 0.9f, 0.9f));
      }
      if (line_drawing) {
        g_debugDraw.DrawSegment(points.back(), m_mouseWorld,
                                b2Color(0.9f, 0.9f, 0.9f));
      }
    }
  }
  void UpdateUI() override {
    ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
    ImGui::SetNextWindowSize(ImVec2(285.0f, 285.0f));
    ImGui::Begin("Map Creator Controls", nullptr, ImGuiWindowFlags_NoResize);

    if (ImGui::Button("Draw Polygon")) {
      points.clear();
      line_drawing = false;
      current_mode = DRAW_POLYGON;
    }
    if (ImGui::Button("Draw Line")) {
      points.clear();
      line_drawing = false;
      current_mode = DRAW_LINE;
    }
    if (ImGui::Button("Draw Hollow Polygon")) {
      points.clear();
      line_drawing = false;
      current_mode = DRAW_HOLLOW_POLYGON;
    }
    if (ImGui::Button("Draw Circle")) {
      points.clear();
      line_drawing = false;
      current_mode = DRAW_CIRCLE;
    }
    if (ImGui::Button("Draw None")) {
      points.clear();
      line_drawing = false;
      current_mode = DRAW_NONE;
    }

    ImGui::Separator();
    if (ImGui::Button("Drone Spawn")) {
      points.clear();
      line_drawing = false;
      current_mode = DRAW_DRONE_SPAWN;
    }

    ImGui::End();
  }
};