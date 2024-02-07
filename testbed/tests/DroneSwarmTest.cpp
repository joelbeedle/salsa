#include <box2d/box2d.h>
#include <stdio.h>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "BehaviourFactory.h"
#include "BehaviourTypes.h"
#include "Drone.h"
#include "FlockingBehaviour.h"
#include "ObjectTypes.h"
#include "PheremoneBehaviour.h"
#include "Tree.h"
#include "draw.h"
#include "imgui/imgui.h"
#include "test.h"

#define DRONE_COUNT 50
#define TREE_COUNT 500
#define BORDER_WIDTH 100.0f
#define BORDER_HEIGHT 100.0f

std::vector<Tree *> tMappedTrees;
class DroneContactListener : public b2ContactListener {
  bool getDroneAndTree(b2Contact *contact, b2Fixture *&drone,
                       b2Fixture *&tree) {
    b2Fixture *fixtureA = contact->GetFixtureA();
    b2Fixture *fixtureB = contact->GetFixtureB();

    bool sensorA = fixtureA->IsSensor();
    bool sensorB = fixtureB->IsSensor();
    if (!(sensorA ^ sensorB)) {
      return false;
    };

    // b2Body* entityA = fixtureA->GetBody();
    // b2Body* entityB = fixtureB->GetBody();

    if (sensorB) {  // fixtureB must be a done
      tree = fixtureA;
      drone = fixtureB;
    } else {  // fixtureA must be a drone
      tree = fixtureB;
      drone = fixtureA;
    }

    return true;
  }
  void BeginContact(b2Contact *contact) override {
    b2Fixture *fixtureA = contact->GetFixtureA();
    b2Fixture *fixtureB = contact->GetFixtureB();

    // Check if one of the fixtures is a sensor and the other is a tree
    if (fixtureA->IsSensor() &&
        fixtureB->GetFilterData().categoryBits == 0x0002) {
      // fixtureA is the drone's sensor, fixtureB is the tree
      UserData *userData =
          reinterpret_cast<UserData *>(fixtureB->GetUserData().pointer);
      Tree *tree = userData->tree;
      tree->setMapped(true);
    } else if (fixtureB->IsSensor() &&
               fixtureA->GetFilterData().categoryBits == 0x0002) {
      // fixtureB is the drone's sensor, fixtureA is the tree
      UserData *userData =
          reinterpret_cast<UserData *>(fixtureA->GetUserData().pointer);
      Tree *tree = userData->tree;
      tree->setMapped(true);
    }
  }

  void EndContact(b2Contact *contact) override {
    // Handle end of contact if needed
  }
};
class MyDraw : public b2Draw {
 private:
  std::vector<Tree *> allTrees;
  std::vector<Drone *> allDrones;

 public:
  void setTrees(std::vector<Tree *> trees) { allTrees = trees; }

  void DrawPolygon(const b2Vec2 *vertices, int32 vertexCount,
                   const b2Color &color) override {
    // Use default debug draw for polygons
    g_debugDraw.DrawPolygon(vertices, vertexCount, color);
  }

  void DrawSolidPolygon(const b2Vec2 *vertices, int32 vertexCount,
                        const b2Color &color) override {
    // Use default debug draw for solid polygons
    g_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);
  }

  void DrawCircle(const b2Vec2 &center, float radius,
                  const b2Color &color) override {
    // Default debug draw for other circles
    g_debugDraw.DrawCircle(center, radius, color);
  }

  void DrawSolidCircle(const b2Vec2 &center, float radius, const b2Vec2 &axis,
                       const b2Color &color) override {
    g_debugDraw.DrawSolidCircle(center, radius, axis, color);
  }

  void DrawSegment(const b2Vec2 &p1, const b2Vec2 &p2,
                   const b2Color &color) override {
    g_debugDraw.DrawSegment(p1, p2, color);
  }

  void DrawTransform(const b2Transform &xf) override {
    g_debugDraw.DrawTransform(xf);
  }

  void DrawPoint(const b2Vec2 &p, float size, const b2Color &color) override {
    g_debugDraw.DrawPoint(p, size, color);
  }
};

void createBounds(b2World *world) {
  // Define the ground body.
  b2BodyDef groundBodyDef;
  groundBodyDef.position.Set(0.0f, 0.0f);

  b2Body *groundBody = world->CreateBody(&groundBodyDef);

  b2EdgeShape groundBox;

  // bottom
  groundBox.SetTwoSided(b2Vec2(0.0f, 0.0f), b2Vec2(BORDER_WIDTH, 0.0f));
  groundBody->CreateFixture(&groundBox, 0.0f);

  // top
  groundBox.SetTwoSided(b2Vec2(0.0f, BORDER_HEIGHT),
                        b2Vec2(BORDER_WIDTH, BORDER_HEIGHT));
  groundBody->CreateFixture(&groundBox, 0.0f);

  // left
  groundBox.SetTwoSided(b2Vec2(0.0f, 0.0f), b2Vec2(0.0f, BORDER_HEIGHT));
  groundBody->CreateFixture(&groundBox, 0.0f);

  // right
  groundBox.SetTwoSided(b2Vec2(BORDER_WIDTH, 0.0f),
                        b2Vec2(BORDER_WIDTH, BORDER_HEIGHT));
  groundBody->CreateFixture(&groundBox, 0.0f);
}

class DroneSwarmTest : public Test {
 public:
  // Behaviours
  SwarmBehaviour *behaviour;
  BehaviourType currentBehaviourType;
  MyDraw myDraw;
  DroneContactListener droneContactListener;

  // Parameters
  FlockingParameters flockingParams = {50.0f, 1.0f, 1.0f, 1.0f, 1.0f};
  PheremoneParameters pheremoneParams = {0.1f};

  // Drone settings
  std::vector<Drone *> drones;

  float viewRange = 10.0f;
  float maxSpeed = 10.0f;
  float maxForce = 0.3f;

  // Tree settings
  std::vector<Tree *> trees;
  std::vector<Tree *> mappedTrees;

  // ImGUI settings to switch behaviours
  int currentBehaviourIndex;  // Index of the currently selected behaviour

  // IMPORTANT: UPDATE WHEN ADDING BEHAVIOURS
  std::vector<BehaviourType> indexToBehaviourType = {
      BehaviourType::Flocking,  // 0
      BehaviourType::Pheremone  // 1
  };
  const std::vector<BehaviourTypeInfo> behaviourTypes = {
      {BehaviourType::Flocking, "flockingBehaviour"},
      {BehaviourType::Pheremone, "pheremoneBehaviour"}};
  const char *behaviours[2] = {"flockingBehaviour", "pheremoneBehaviour"};

  // Visual settings
  bool drawDroneVisualRange = false;

 public:
  DroneSwarmTest() {
    {
      initWorld();
      m_world->SetContactListener(&droneContactListener);

      initDefaultParameters();

      // Set initial behaviour
      currentBehaviourType = BehaviourType::Flocking;
      currentBehaviourIndex = 0;

      // Set behaviour to equal our current behaviour
      UpdateBehaviour();
      //  Create our drones with our selected behaviour
      createDrones(behaviour);
      createTrees();
      myDraw.setTrees(trees);
    }
  }

  void initWorld() {
    createBounds(m_world);
    myDraw.SetFlags(b2Draw::e_shapeBit | b2Draw::e_jointBit);
    m_world->SetDebugDraw(&myDraw);
    b2Vec2 gravity(0.0f, 0.0f);
    m_world->SetGravity(gravity);
  }

  void initDefaultParameters() {
    viewRange = 5.0f;
    maxSpeed = 10.0f;
    maxForce = 0.3f;

    flockingParams = {50.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    pheremoneParams = {0.1f};
  }

  void createDrones(SwarmBehaviour *b) {
    const float margin = 2.0f;  // Define a margin to prevent spawning exactly
                                // at the border or outside
    for (int i = 0; i < DRONE_COUNT; i++) {
      float x = (rand() % static_cast<int>(BORDER_WIDTH - 2 * margin)) + margin;
      float y =
          (rand() % static_cast<int>(BORDER_HEIGHT - 2 * margin)) + margin;
      drones.push_back(new Drone(m_world, b2Vec2(x, y), behaviour, viewRange,
                                 maxSpeed, maxForce, 1.0f));
    }
  }

  void createTrees() {
    const float margin = 2.0f;
    for (int i = 0; i < TREE_COUNT; i++) {
      float x = (rand() % static_cast<int>(BORDER_WIDTH - 2 * margin)) + margin;
      float y =
          (rand() % static_cast<int>(BORDER_HEIGHT - 2 * margin)) + margin;
      trees.push_back(new Tree(m_world, b2Vec2(x, y), false, false, 1.0f));
    }
  }

  void DestroyDrones() {
    for (auto &drone : drones) {
      m_world->DestroyBody(drone->getBody());
    }
    drones.clear();
  }

  void SetBehaviour() {
    for (auto &drone : drones) {
      drone->setBehaviour(behaviour);
    }
  }

  void UpdateDroneSettings() {
    // Update drone settings:
    for (auto &drone : drones) {
      drone->setMaxForce(maxForce);
      drone->setMaxSpeed(maxSpeed);
      drone->setViewRange(viewRange);
      drone->updateSensorRange();
    }
  }

  void UpdateBehaviour() {
    currentBehaviourType = behaviourTypes[currentBehaviourIndex].type;
    SwarmBehaviour *newBehaviour;
    switch (currentBehaviourType) {
      case (BehaviourType::Flocking):
        newBehaviour = BehaviourFactory::createBehaviour(
            BehaviourType::Flocking, flockingParams);
        break;
      case (BehaviourType::Pheremone):
        newBehaviour = BehaviourFactory::createBehaviour(
            BehaviourType::Pheremone, pheremoneParams);
        break;
    }
    // delete behaviour;
    behaviour = newBehaviour;
  }

  void UpdateUI() override {
    ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
    ImGui::SetNextWindowSize(ImVec2(285.0f, 285.0f));
    ImGui::Begin("Swarm Controls", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

    // Dropdown to select the behaviour
    if (ImGui::BeginCombo(
            "Behaviour",
            behaviourTypes[currentBehaviourIndex].displayName.c_str())) {
      for (int i = 0; i < behaviourTypes.size(); i++) {
        bool isSelected = (currentBehaviourIndex == i);
        if (ImGui::Selectable(behaviourTypes[i].displayName.c_str(),
                              isSelected)) {
          currentBehaviourIndex = i;
          UpdateBehaviour();
          SetBehaviour();
        }
        if (isSelected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }

    // Drone settings
    ImGui::Text("Drone Settings");
    bool droneChanged = false;
    droneChanged |= ImGui::SliderFloat("maxSpeed", &maxSpeed, 0.0f, 50.0f);
    droneChanged |= ImGui::SliderFloat("maxForce", &maxForce, 0.0f, 10.0f);
    droneChanged |= ImGui::SliderFloat("viewRange", &viewRange, 0.0f, 100.0f);

    if (droneChanged) {
      UpdateDroneSettings();
    }

    // Display different behaviour settings depending on the behaviour selected
    ImGui::Text("Behaviour Settings");
    bool changed = false;
    for (auto &[name, parameter] : behaviour->getParameters()) {
      changed |= ImGui::SliderFloat(name.c_str(), parameter.value,
                                    parameter.minSetting, parameter.maxSetting);
    }

    if (changed) {
      // UpdateBehaviour();
      SetBehaviour();
    }

    // Visual settings
    ImGui::Text("Visual Settings");
    ImGui::Checkbox("Draw Drone visual range", &drawDroneVisualRange);

    if (ImGui::Button("Reset Simulation")) {
      DestroyDrones();
      initDefaultParameters();
      createDrones(behaviour);
    }

    ImGui::End();
  }

  static Test *Create() { return new DroneSwarmTest; }

  void ManualDebugDraw(b2World *world, MyDraw *debugDraw) {
    for (b2Body *body = world->GetBodyList(); body; body = body->GetNext()) {
      const b2Transform &transform = body->GetTransform();

      for (b2Fixture *fixture = body->GetFixtureList(); fixture;
           fixture = fixture->GetNext()) {
        if (fixture->IsSensor()) {
          uint16 categoryBits = fixture->GetFilterData().categoryBits;
          if (categoryBits == 0x0001) {
            // This is a drone sensor, draw if wanted
            if (drawDroneVisualRange) {
              const b2CircleShape *circleShape =
                  static_cast<const b2CircleShape *>(fixture->GetShape());
              b2Vec2 position =
                  transform.p + b2Mul(transform.q, circleShape->m_p);
              debugDraw->DrawCircle(position, circleShape->m_radius,
                                    b2Color(0.5f, 0.5f, 0.5f));
            }
            // skip this fixture as it's been dealt with
            continue;
          }
          if (categoryBits == 0x0002) {
            // This is fixture tree sensor, but we deal with trees individually
            // later
          }
        }

        // Check if the fixture has user data, if it does its a drone (non
        // sensor) or tree (sensor) fixture
        if (fixture->GetUserData().pointer != 0) {
          UserData *userData =
              reinterpret_cast<UserData *>(fixture->GetUserData().pointer);
          if (userData == nullptr) {
            std::cout << "User data is null" << std::endl;
            continue;
          }
          // Depending on the type, draw the object
          switch (userData->type) {
            case ObjectType::Drone: {
              Drone *drone = userData->drone;
              // Draw drone
              b2Vec2 position = body->GetPosition();
              debugDraw->DrawSolidCircle(position, drone->getRadius(),
                                         transform.q.GetXAxis(),
                                         b2Color(0.7f, 0.5f, 0.5f));
              break;
            }
            case ObjectType::Tree: {
              Tree *tree = userData->tree;
              b2Vec2 position = body->GetPosition();
              const b2CircleShape *circleShape =
                  static_cast<const b2CircleShape *>(fixture->GetShape());

              if (tree->isMapped()) {
                b2Color customColor =
                    b2Color(0.0f, 1.0f, 0.0f);  // Custom color for mapped trees
                g_debugDraw.DrawSolidCircle(position, circleShape->m_radius,
                                            transform.q.GetXAxis(),
                                            customColor);
              } else {
                b2Color customColor =
                    b2Color(1.0f, 0.0f, 0.0f);  // Custom color for mapped trees
                g_debugDraw.DrawSolidCircle(position, circleShape->m_radius,
                                            transform.q.GetXAxis(),
                                            customColor);
              }
              break;
            }
          }
          continue;
        }

        // Draw everything else that's not anything above with default values
        switch (fixture->GetType()) {
          case b2Shape::e_circle: {
            const b2CircleShape *circleShape =
                static_cast<const b2CircleShape *>(fixture->GetShape());
            b2Vec2 position =
                transform.p + b2Mul(transform.q, circleShape->m_p);
            debugDraw->DrawSolidCircle(position, circleShape->m_radius,
                                       transform.q.GetXAxis(),
                                       b2Color(1.0f, 0.5f, 0.5f));
            break;
          }
          case b2Shape::e_polygon: {
            const b2PolygonShape *polygonShape =
                static_cast<const b2PolygonShape *>(fixture->GetShape());
            b2Vec2 vertices[b2_maxPolygonVertices];
            for (int i = 0; i < polygonShape->m_count; ++i) {
              vertices[i] = b2Mul(transform, polygonShape->m_vertices[i]);
            }
            debugDraw->DrawSolidPolygon(vertices, polygonShape->m_count,
                                        b2Color(0.5f, 0.5f, 0.5f));
            break;
          }
          case b2Shape::e_edge: {
            const b2EdgeShape *edgeShape =
                static_cast<const b2EdgeShape *>(fixture->GetShape());
            b2Vec2 v1 = b2Mul(transform, edgeShape->m_vertex1);
            b2Vec2 v2 = b2Mul(transform, edgeShape->m_vertex2);
            debugDraw->DrawSegment(v1, v2, b2Color(0.5f, 1.0f, 0.5f));
            break;
          }
          case b2Shape::e_chain: {
            const b2ChainShape *chainShape =
                static_cast<const b2ChainShape *>(fixture->GetShape());
            int32 count = chainShape->m_count;
            const b2Vec2 *vertices = chainShape->m_vertices;
            b2Vec2 v1 = b2Mul(transform, vertices[0]);
            for (int32 i = 1; i < count; ++i) {
              b2Vec2 v2 = b2Mul(transform, vertices[i]);
              debugDraw->DrawSegment(v1, v2, b2Color(0.5f, 0.5f, 0.5f));
              debugDraw->DrawCircle(v1, 0.05f, b2Color(0.5f, 0.5f, 0.5f));
              v1 = v2;
            }
            break;
          }
          default:
            // Shouldn't reach here
            break;
        }
      }
    }
  }

  void Step(Settings &settings) override {
    Test::Step(settings);

    // m_world->DebugDraw();
    ManualDebugDraw(m_world, &myDraw);
    // Update Drone position
    for (auto &drone : drones) {
      drone->update(drones);
    }

    int mappedTreeCount = tMappedTrees.size();
    // std::cout << mappedTreeCount << std::endl;
  }
};

static int testIndex =
    RegisterTest("Swarm", "DroneSwarmTest", DroneSwarmTest::Create);
