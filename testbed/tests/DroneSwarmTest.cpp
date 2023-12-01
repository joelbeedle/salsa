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
#include "PheremoneBehaviour.h"
#include "Tree.h"
#include "draw.h"
#include "imgui/imgui.h"
#include "test.h"

#define DRONE_COUNT 50
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
      Tree *tree = reinterpret_cast<Tree *>(fixtureB->GetUserData().pointer);
      tree->setMapped(true);
    } else if (fixtureB->IsSensor() &&
               fixtureA->GetFilterData().categoryBits == 0x0002) {
      // fixtureB is the drone's sensor, fixtureA is the tree
      Tree *tree = reinterpret_cast<Tree *>(fixtureA->GetUserData().pointer);
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
    constexpr float epsilon = 0.01f;  // Tolerance for position comparison
    std::cout << "Drawing SolidCircle at " << center.x << ", " << center.y
              << std::endl;
    for (const auto &tree : allTrees) {
      b2Vec2 treePos = tree->getBody()->GetPosition();

      // Use a tolerance value for comparing positions
      if ((treePos - center).LengthSquared() <= epsilon * epsilon) {
        if (tree->isMapped()) {
          b2Color customColor =
              b2Color(0.0f, 1.0f, 0.0f);  // Custom color for mapped trees
          g_debugDraw.DrawSolidCircle(center, radius, axis, customColor);
          return;
        } else {
          b2Color customColor =
              b2Color(1.0f, 0.0f, 0.0f);  // Custom color for mapped trees
          g_debugDraw.DrawSolidCircle(center, radius, axis, customColor);
          return;
        }
      }
    }
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
    for (int i = 0; i < DRONE_COUNT; i++) {
      drones.push_back(
          new Drone(m_world,
                    b2Vec2(rand() % static_cast<int>(BORDER_WIDTH),
                           rand() % static_cast<int>(BORDER_HEIGHT)),
                    behaviour, viewRange, maxSpeed, maxForce, 1.0f));
    }
  }

  void createTrees() {
    for (int i = 0; i < 100; i++) {
      trees.push_back(new Tree(m_world,
                               b2Vec2(rand() % static_cast<int>(BORDER_WIDTH),
                                      rand() % static_cast<int>(BORDER_HEIGHT)),
                               false, false, 1.0f));
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

    if (ImGui::Button("Reset Simulation")) {
      DestroyDrones();
      initDefaultParameters();
      createDrones(behaviour);
    }

    ImGui::End();
  }

  static Test *Create() { return new DroneSwarmTest; }

  void Step(Settings &settings) override {
    Test::Step(settings);

    m_world->DebugDraw();

    // Update Drone position
    for (auto &drone : drones) {
      drone->update(drones);
    }

    int mappedTreeCount = tMappedTrees.size();
    std::cout << mappedTreeCount << std::endl;
  }
};

static int testIndex =
    RegisterTest("Swarm", "DroneSwarmTest", DroneSwarmTest::Create);
