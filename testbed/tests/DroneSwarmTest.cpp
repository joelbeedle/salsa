#include <box2d/box2d.h>
#include <stdio.h>

#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
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
#define TREE_COUNT 10000
#define BORDER_WIDTH 2000.0f
#define BORDER_HEIGHT 2000.0f
#define MAX_TIME 100000.0f

struct DroneParameters {
  float cameraViewRange;
  float obstacleViewRange;
  float droneDetectionRange;
  float maxSpeed;
  float maxForce;
  float mass;
  float radius;
};

void CreatePreset(std::unordered_map<std::string, DroneParameters> &presets,
                  const std::string &name, const DroneParameters &params) {
  presets[name] = params;
}

void DeletePreset(std::unordered_map<std::string, DroneParameters> &presets,
                  const std::string &name) {
  presets.erase(name);
}

void UpdatePreset(std::unordered_map<std::string, DroneParameters> &presets,
                  const std::string &name, const DroneParameters &params) {
  presets[name] = params;
}

class DroneSwarmTest : public Test {
 public:
  // Lists
  std::vector<Tree *> foundDiseasedTrees;
  std::vector<Tree *> actualDiseasedTrees;
  std::vector<b2Vec2 *> foundDiseasedTreePositions;
  std::set<b2Vec2 *> foundDiseaseadTreePositionsSet;
  std::vector<b2Vec2 *> actualDiseasedTreePositions;

  std::unordered_map<std::string, DroneParameters> dronePresets;

  // Behaviours
  SwarmBehaviour *behaviour;
  std::string currentBehaviourName;
  DroneContactListener droneContactListener;

  // Parameters
  FlockingParameters flockingParams;
  PheremoneParameters pheremoneParams;
  PSOParameters psoParams;
  UniformRandomWalkParameters uniformRandomWalkParams;

  // Drone settings
  std::vector<Drone *> drones;

  float obstacleViewRange;
  float cameraViewRange;
  float maxSpeed;
  float maxForce;

  DroneParameters djiMatrice300RTK = {8.0f, 40.0f, 2000.0f, 17.0f,
                                      0.3f, 6.3f,  0.45f};
  DroneParameters djiPhantom4RTK = {7.0f, 40.0f, 2000.0f, 13.0f,
                                    0.3f, 1.4f,  0.35f};
  DroneParameters smallDrone = {7.0f, 40.0f, 2000.0f, 10.0f, 0.3f, 1.5f, 0.10f};
  std::string currentPresetName;
  DroneParameters *droneParams;

  // Tree settings
  std::vector<Tree *> trees;
  std::vector<Tree *> mappedTrees;

  // Visual settings
  bool drawDroneVisualRange = false;
  bool drawTrees = true;
  bool firstRun = true;
  std::vector<b2Vec2> treePositions;
  std::vector<b2Color> treeColors;

  b2Color falseColour =
      b2Color(0.5f * 0.95294f, 0.5f * 0.50588f, 0.5f * 0.50588f, 0.5f * 0.25f);
  b2Color trueColour =
      b2Color(0.5f * 0.77f, 0.5f * 0.92f, 0.5f * 0.66f, 0.5f * 0.25f);

  // Data collection settings
  bool testRunning = false;
  float time = 0.0f;
  int iters = 0;
  float timestep;

 public:
  DroneSwarmTest() {
    {
      initWorld();
      m_world->SetContactListener(&droneContactListener);
      g_camera.m_center.x = BORDER_HEIGHT / 2;
      g_camera.m_center.y = BORDER_WIDTH / 2;
      g_camera.m_zoom = 10.0f;
      initDefaultParameters();
      initDefaultBehaviours();
      createDronesCircular(behaviour, droneParams);
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
    // based off of drone Matrice 300 RTK
    CreatePreset(dronePresets, "Matrice 300 RTK", djiMatrice300RTK);
    CreatePreset(dronePresets, "Phantom 4 RTK", djiPhantom4RTK);
    CreatePreset(dronePresets, "Small Drone", smallDrone);
    currentPresetName = dronePresets.begin()->first;
    droneParams = &dronePresets[currentPresetName];

    cameraViewRange = 8.0f;
    obstacleViewRange = 40.0f;
    maxSpeed = 17.0f;
    maxForce = 0.3f;

    flockingParams = {50.0f, 1.6f, 0.8f, 1.8f, 3.0f};
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

  void createDrones(SwarmBehaviour *b, DroneParameters *params) {
    const float margin = 2.0f;  // Define a margin to prevent spawning exactly
                                // at the border or outside
    for (int i = 0; i < DRONE_COUNT; i++) {
      float x = (rand() % static_cast<int>(BORDER_WIDTH - 2 * margin)) + margin;
      float y =
          (rand() % static_cast<int>(BORDER_HEIGHT - 2 * margin)) + margin;
      drones.push_back(new Drone(
          m_world, b2Vec2(x, y), behaviour, params->cameraViewRange,
          params->obstacleViewRange, params->maxSpeed, params->maxForce,
          params->radius, params->mass, params->droneDetectionRange));
    }
  }

  void createDronesCircular(SwarmBehaviour *b, DroneParameters *params) {
    // Calculate the total area needed for all drones
    float droneArea = M_PI * std::pow(params->radius, 2);
    float totalDroneArea = DRONE_COUNT * droneArea;

    // Calculate the radius of the circle needed to fit all drones
    float requiredCircleRadius = sqrt(totalDroneArea / M_PI);

    // Adjust center of the circle to be the center of the map
    float centerX = BORDER_WIDTH / 2.0f;
    float centerY = BORDER_HEIGHT / 2.0f;

    for (int i = 0; i < DRONE_COUNT; i++) {
      // Generate random angle and radius within the required circle
      float theta = static_cast<float>(rand()) / RAND_MAX * 2.0f * M_PI;
      // Ensure drones fit within the required circle, leaving a margin equal to
      // the drone's radius
      float r = sqrt(static_cast<float>(rand()) / RAND_MAX) *
                (requiredCircleRadius - params->radius);

      // Convert polar coordinates (r, theta) to Cartesian coordinates (x, y)
      float x = centerX + r * cos(theta);
      float y = centerY + r * sin(theta);

      drones.push_back(new Drone(
          m_world, b2Vec2(x, y), behaviour, params->cameraViewRange,
          params->obstacleViewRange, params->maxSpeed, params->maxForce,
          params->radius, params->mass, params->droneDetectionRange));
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
    // Seed for reproducability
    srand(1967);
    const float margin = 2.0f;
    for (int i = 0; i < TREE_COUNT; i++) {
      float x = (rand() % static_cast<int>(BORDER_WIDTH - 2 * margin)) + margin;
      float y =
          (rand() % static_cast<int>(BORDER_HEIGHT - 2 * margin)) + margin;
      trees.push_back(new Tree(m_world, i, b2Vec2(x, y), false, false, 2.5f));
      treePositions.push_back(trees[i]->getBody()->GetPosition());
      treeColors.push_back(falseColour);
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
      drone->setMaxForce(droneParams->maxForce);
      drone->setMaxSpeed(droneParams->maxSpeed);
      drone->setViewRange(droneParams->cameraViewRange);
      drone->setObstacleViewRange(droneParams->obstacleViewRange);
      drone->updateSensorRange();
    }
  }

  void ResetTrees() {
    treeColors.clear();
    for (auto &tree : trees) {
      tree->setMapped(false);
      treeColors.push_back(falseColour);
    }
  }

  void BeginSimulation() {
    testRunning = true;
    time = 0.0f;
    iters = 0;
    DestroyDrones();
    createDronesCircular(behaviour, droneParams);
    ResetTrees();
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
    ImGui::Separator();

    ImGui::Text("Behaviour Settings");
    bool changed = false;
    for (auto &[name, parameter] : behaviour->getParameters()) {
      changed |= ImGui::SliderFloat(name.c_str(), parameter.value,
                                    parameter.minSetting, parameter.maxSetting);
    }

    if (changed) {
      SetBehaviour();
    }
    ImGui::Separator();

    // Visual settings
    ImGui::Text("Visual Settings");
    ImGui::Checkbox("Draw Drone visual range", &drawDroneVisualRange);
    ImGui::Checkbox("Draw Trees", &drawTrees);

    if (ImGui::Button("Reset Simulation")) {
      DestroyDrones();
      createDronesCircular(behaviour, droneParams);
      ResetTrees();
    }

    ImGui::End();

    // Drone settings window
    ImGui::SetNextWindowPos(ImVec2(10.0f, 400.0f));
    ImGui::SetNextWindowSize(ImVec2(285.0f, 285.0f));
    ImGui::Begin("Drone Settings", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
    if (ImGui::BeginCombo("Drone Preset", currentPresetName.c_str())) {
      for (auto &[name, params] : dronePresets) {
        bool isSelected = (currentPresetName == name);
        if (ImGui::Selectable(name.c_str(), isSelected)) {
          currentPresetName = name;
          droneParams = &dronePresets[name];
          std::vector<Drone *> newDrones;
          for (auto &drone : drones) {
            newDrones.push_back(
                new Drone(m_world, drone->getBody()->GetPosition(), behaviour,
                          droneParams->cameraViewRange,
                          droneParams->obstacleViewRange, droneParams->maxSpeed,
                          droneParams->maxForce, droneParams->radius,
                          droneParams->mass, droneParams->droneDetectionRange));
            m_world->DestroyBody(drone->getBody());
          }
          drones = newDrones;
        }
        if (isSelected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    ImGui::Separator();

    ImGui::Text("Drone Preset Settings");
    bool droneChanged = false;
    droneChanged |=
        ImGui::SliderFloat("maxSpeed", &droneParams->maxSpeed, 0.0f, 50.0f);
    droneChanged |=
        ImGui::SliderFloat("maxForce", &droneParams->maxForce, 0.0f, 10.0f);
    droneChanged |= ImGui::SliderFloat(
        "cameraViewRange", &droneParams->cameraViewRange, 0.0f, 100.0f);
    droneChanged |= ImGui::SliderFloat(
        "obstacleViewRange", &droneParams->obstacleViewRange, 0.0f, 100.0f);

    if (droneChanged) {
      UpdateDroneSettings();
    }

    ImGui::Separator();
    if (ImGui::Button("Run Simulation")) {
      if (!testRunning) {
        BeginSimulation();
      } else {
        testRunning = false;
        BeginSimulation();
      }
    }

    ImGui::End();
  }

  static Test *Create() { return new DroneSwarmTest; }

  void Draw(b2World *world, DebugDraw *debugDraw,
            std::vector<int> foundTreeIDs) {
    if (drawTrees) {
      if (firstRun) {
        debugDraw->DrawAllTrees(treePositions, treeColors);
        firstRun = false;
      } else {
        debugDraw->DrawTrees(treePositions, treeColors, foundTreeIDs);
      }
    }

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

  void Log(float time, int iters) {
    if (iters == 1) {
      std::cout << "=======================================" << std::endl;
      std::cout << "Starting simulation" << std::endl;
      std::cout << "=======================================" << std::endl;
      std::cout << "Drone count: " << DRONE_COUNT << std::endl;
      std::cout << "Tree count: " << TREE_COUNT << std::endl;
      std::cout << "Area: " << BORDER_WIDTH << "x" << BORDER_HEIGHT
                << std::endl;
      std::cout << "=======================================" << std::endl;
      std::cout << "Using Behaviour: " << currentBehaviourName << std::endl;
      std::cout << "Behaviour settings: " << std::endl;
      for (auto &[name, parameter] : behaviour->getParameters()) {
        std::cout << name << ": " << *parameter.value << std::endl;
      }
      std::cout << "=======================================" << std::endl;
      std::cout << "Using Preset: " << currentPresetName << std::endl;
      std::cout << "Drone count: " << DRONE_COUNT << std::endl;
      std::cout << "=======================================" << std::endl;
      std::cout << "Time (s) | Trees mapped | % mapped of total" << std::endl;
    }
    if (iters == 1 || iters % 300 == 0) {
      int totalMapped = 0;
      for (auto &tree : trees) {
        if (tree->isMapped()) {
          totalMapped++;
        }
      }
      float percentage =
          (static_cast<float>(totalMapped) / static_cast<float>(trees.size())) *
          100.0f;
      std::cout << time << " | " << totalMapped << " | " << percentage << "%"
                << std::endl;
      if (time >= MAX_TIME || totalMapped == TREE_COUNT) {
        std::cout << "=======================================" << std::endl;
        std::cout << "Simulation complete" << std::endl;
        std::cout << "Time taken: " << time << "s" << std::endl;
        std::cout << "=======================================" << std::endl;
      }
    }
  }
  void Step(Settings &settings) override {
    Test::Step(settings);
    time += 1.0f / 30.0f;
    iters += 1;
    std::vector<Tree *> foundTrees;
    std::vector<int> foundIDs;
    foundTrees.reserve(3 * drones.size());

    //  Update Drone position and add found trees to list
    for (auto &drone : drones) {
      drone->update(drones);
      auto &found = drone->getFoundDiseasedTrees();
      foundTrees.insert(foundTrees.end(), found.begin(), found.end());
      drone->clearLists();
    }

    // Update newly found trees with their correct colours
    for (Tree *tree : foundTrees) {
      treeColors[tree->getID()] = trueColour;
    }

    // Draw world
    Draw(m_world, &g_debugDraw, foundIDs);
    // Logging
    if (testRunning) {
      Log(time, iters);
    }
  }
};

static int testIndex =
    RegisterTest("Swarm", "DroneSwarmTest", DroneSwarmTest::Create);
