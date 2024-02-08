#include <box2d/box2d.h>
#include <stdio.h>

#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "Drone.h"
#include "DroneContactListener.h"
#include "FlockingBehaviour.h"
#include "ObjectTypes.h"
#include "PSOBehaviour.h"
#include "PheremoneBehaviour.h"
#include "SwarmBehaviourRegistry.h"
#include "Tree.h"
#include "UniformRandomWalkBehaviour.h"
#include "draw.h"
#include "imgui/imgui.h"
#include "settings.h"
#include "test.h"

#define DRONE_COUNT 50
#define TREE_COUNT 1000
#define BORDER_WIDTH 100.0f
#define BORDER_HEIGHT 100.0f

class DroneSwarmTest : public Test {
 public:
  // Lists
  std::vector<Tree *> foundDiseasedTrees;
  std::vector<Tree *> actualDiseasedTrees;
  std::vector<b2Vec2 *> foundDiseasedTreePositions;
  std::set<b2Vec2 *> foundDiseaseadTreePositionsSet;
  std::vector<b2Vec2 *> actualDiseasedTreePositions;

  // Behaviours
  SwarmBehaviour *behaviour;
  std::string currentBehaviourName;
  DroneContactListener droneContactListener;

  // Parameters
  FlockingParameters flockingParams = {50.0f, 1.0f, 1.0f, 1.0f, 1.0f};
  PheremoneParameters pheremoneParams = {0.1f, 1.0f};
  PSOParameters psoParams = {0.5f, 0.5f, 0.9f};
  UniformRandomWalkParameters uniformRandomWalkParams = {5.0f, 0.5f, 1.0f,
                                                         1.0 / 60.0f};

  // Drone settings
  std::vector<Drone *> drones;

  float viewRange = 10.0f;
  float maxSpeed = 10.0f;
  float maxForce = 0.3f;

  // Tree settings
  std::vector<Tree *> trees;
  std::vector<Tree *> mappedTrees;

  // Visual settings
  bool drawDroneVisualRange = false;

 public:
  DroneSwarmTest() {
    {
      initWorld();
      m_world->SetContactListener(&droneContactListener);

      initDefaultParameters();
      initDefaultBehaviours();
      createDrones(behaviour);
      createTrees();
    }
  }

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

  void initWorld() {
    createBounds(m_world);
    g_debugDraw.SetFlags(b2Draw::e_shapeBit | b2Draw::e_jointBit);
    // m_world->SetDebugDraw(&myDraw);
    b2Vec2 gravity(0.0f, 0.0f);
    m_world->SetGravity(gravity);
  }

  void initDefaultParameters() {
    viewRange = 5.0f;
    maxSpeed = 10.0f;
    maxForce = 0.3f;

    flockingParams = {50.0f, 1.6f, 0.8f, 1.6f, 1.2f};
    pheremoneParams = {0.1f, 1.0f};
    uniformRandomWalkParams = {5.0f, 0.5f, 1.0f, 1.0 / 60.0f};
  }

  void initDefaultBehaviours() {
    std::unique_ptr<FlockingBehaviour> flockBehaviour =
        std::make_unique<FlockingBehaviour>(flockingParams);
    std::unique_ptr<PheremoneBehaviour> pheremoneBehaviour =
        std::make_unique<PheremoneBehaviour>(pheremoneParams);
    std::unique_ptr<PSOBehaviour> psoBehaviour =
        std::make_unique<PSOBehaviour>(psoParams);
    std::unique_ptr<UniformRandomWalkBehaviour> uniformRandomWalkBehaviour =
        std::make_unique<UniformRandomWalkBehaviour>(uniformRandomWalkParams);

    SwarmBehaviourRegistry::getInstance().add("PheremoneBehaviour",
                                              std::move(pheremoneBehaviour));

    SwarmBehaviourRegistry::getInstance().add("PSOBehaviour",
                                              std::move(psoBehaviour));

    SwarmBehaviourRegistry::getInstance().add("FlockingBehaviour",
                                              std::move(flockBehaviour));
    SwarmBehaviourRegistry::getInstance().add(
        "UniformRandomWalkBehaviour", std::move(uniformRandomWalkBehaviour));

    auto &registry = SwarmBehaviourRegistry::getInstance();
    auto behaviorNames = registry.getSwarmBehaviourNames();
    // Set initial behaviour
    if (!behaviorNames.empty()) {
      // Select the first behavior as the default one
      currentBehaviourName = behaviorNames[0];
      behaviour = registry.getSwarmBehaviour(currentBehaviourName);
    }
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

  std::vector<Tree *> getTreesInRadius(const b2Vec2 &position, float radius,
                                       const std::vector<Tree *> &allTrees) {
    std::vector<Tree *> nearbyTrees;
    for (Tree *tree : allTrees) {
      if (b2Distance(position, tree->getBody()->GetPosition()) <= radius) {
        nearbyTrees.push_back(tree);
      }
    }
    return nearbyTrees;
  }

  // Function to update disease spread
  void updateDiseaseSpread(std::vector<Tree *> &allTrees,
                           float infectionRadius) {
    std::vector<Tree *> treesToInfect;

    for (Tree *tree : allTrees) {
      if (tree->isDiseased()) {
        // Get all nearby trees within the infection radius
        std::vector<Tree *> nearbyTrees = getTreesInRadius(
            tree->getBody()->GetPosition(), infectionRadius, allTrees);

        for (Tree *nearbyTree : nearbyTrees) {
          if (!nearbyTree->isDiseased()) {
            // Calculate infection probability based on distance
            float distance = b2Distance(tree->getBody()->GetPosition(),
                                        nearbyTree->getBody()->GetPosition());
            float probabilityOfInfection =
                1.0f - (distance / infectionRadius);  // Example calculation

            // Random chance to infect based on calculated probability
            float randomChance =
                static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
            if (randomChance < probabilityOfInfection) {
              treesToInfect.push_back(nearbyTree);
            }
          }
        }
      }
    }

    // Infect trees determined to be infected in this time step
    for (Tree *treeToInfect : treesToInfect) {
      treeToInfect->setDiseased(true);
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
    updateDiseaseSpread(trees, 20.0f);
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

  void ResetTrees() {
    for (auto &tree : trees) {
      tree->setMapped(false);
    }
  }

  void UpdateUI() override {
    ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
    ImGui::SetNextWindowSize(ImVec2(285.0f, 285.0f));
    ImGui::Begin("Swarm Controls", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

    if (ImGui::BeginCombo("Behaviours", currentBehaviourName.c_str())) {
      auto behaviourNames =
          SwarmBehaviourRegistry::getInstance().getSwarmBehaviourNames();

      for (auto &name : behaviourNames) {
        bool isSelected = (currentBehaviourName == name);
        if (ImGui::Selectable(name.c_str(), isSelected)) {
          currentBehaviourName = name;
          behaviour =
              SwarmBehaviourRegistry::getInstance().getSwarmBehaviour(name);
          SetBehaviour();
        }
        if (isSelected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }

    ImGui::Text("Behaviour Settings");
    bool changed = false;
    for (auto &[name, parameter] : behaviour->getParameters()) {
      changed |= ImGui::SliderFloat(name.c_str(), parameter.value,
                                    parameter.minSetting, parameter.maxSetting);
    }

    if (changed) {
      SetBehaviour();
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

    // Visual settings
    ImGui::Text("Visual Settings");
    ImGui::Checkbox("Draw Drone visual range", &drawDroneVisualRange);

    if (ImGui::Button("Reset Simulation")) {
      DestroyDrones();
      initDefaultParameters();
      createDrones(behaviour);
      ResetTrees();
    }

    ImGui::End();
  }

  static Test *Create() { return new DroneSwarmTest; }

  void Draw(b2World *world, DebugDraw *debugDraw) {
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
    Draw(m_world, &g_debugDraw);
    // Update Drone position
    for (auto &drone : drones) {
      drone->update(drones);
      for (auto &treepos : drone->getFoundDiseasedTreePositions()) {
        foundDiseaseadTreePositionsSet.insert(treepos);
      }
    }
  }
};

static int testIndex =
    RegisterTest("Swarm", "DroneSwarmTest", DroneSwarmTest::Create);
