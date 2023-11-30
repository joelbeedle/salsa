#include <box2d/box2d.h>
#include <stdio.h>

#include "Drone.h"
#include "FlockingBehaviour.h"
#include "PheremoneBehaviour.h"
#include "imgui/imgui.h"
#include "test.h"

#define DRONE_COUNT 50

void createBounds(b2World *world, float screenWidth, float screenHeight) {
  // Define the ground body.
  b2BodyDef groundBodyDef;
  groundBodyDef.position.Set(0.0f, 0.0f);

  b2Body *groundBody = world->CreateBody(&groundBodyDef);

  b2EdgeShape groundBox;

  // bottom
  groundBox.SetTwoSided(b2Vec2(0.0f, 0.0f), b2Vec2(screenWidth, 0.0f));
  groundBody->CreateFixture(&groundBox, 0.0f);

  // top
  groundBox.SetTwoSided(b2Vec2(0.0f, screenHeight),
                        b2Vec2(screenWidth, screenHeight));
  groundBody->CreateFixture(&groundBox, 0.0f);

  // left
  groundBox.SetTwoSided(b2Vec2(0.0f, 0.0f), b2Vec2(0.0f, screenHeight));
  groundBody->CreateFixture(&groundBox, 0.0f);

  // right
  groundBox.SetTwoSided(b2Vec2(screenWidth, 0.0f),
                        b2Vec2(screenWidth, screenHeight));
  groundBody->CreateFixture(&groundBox, 0.0f);
}

class DroneSwarmTest : public Test {
 public:
  SwarmBehaviour *behaviour;
  FlockingBehaviour *flockingBehaviour;
  PheremoneBehaviour *pheremoneBehaviour;

  std::vector<Drone *> drones;
  const float screenWidth = 100.0f;
  const float screenHeight = 100.0f;

  // Floats for Drones
  float viewRange, maxSpeed, maxForce;

  // Floats for FlockingBehaviour
  float separationDistance, alignmentWeight, cohesionWeight, separationWeight,
      obstacleAvoidanceWeight;

  // Floats for PheremoneBehaviour
  float decayRate;

  // Imgui
  const char *behaviours[2] = {"flockingBehaviour", "pheremoneBehaviour"};
  int currentBehaviour;  // Index of the currently selected behaviour

  DroneSwarmTest() {
    {
      // Create Bounds
      createBounds(m_world, screenWidth, screenHeight);
    }
    {
      // Set initial behaviour
      currentBehaviour = 0;  // flockingBehaviour TODO: enum?
      // Initialise world and drones
      b2Vec2 gravity(0.0f, 0.0f);
      m_world->SetGravity(gravity);

      std::vector<b2Body *> obstacles;
      b2Body *currentBody = m_world->GetBodyList();
      while (currentBody) {
        obstacles.push_back(currentBody);
        currentBody = currentBody->GetNext();
      }
      viewRange = 20.0f;
      separationDistance = 50.0f;
      alignmentWeight = 1.0f;
      cohesionWeight = 1.0f;
      separationWeight = 1.0f;
      obstacleAvoidanceWeight = 1.0f;
      maxSpeed = 10.0f;
      maxForce = 0.3f;

      flockingBehaviour = new FlockingBehaviour(
          separationDistance, alignmentWeight, cohesionWeight, separationWeight,
          obstacleAvoidanceWeight);

      decayRate = 0.1f;
      pheremoneBehaviour = new PheremoneBehaviour(decayRate);

      for (int i = 0; i < DRONE_COUNT; i++) {
        drones.push_back(
            new Drone(m_world,
                      b2Vec2(rand() % static_cast<int>(screenWidth),
                             rand() % static_cast<int>(screenHeight)),
                      getSelectedBehaviour(currentBehaviour)));
      }
    }
  }

  void CreateDrones() {
    flockingBehaviour = new FlockingBehaviour(
        separationDistance, alignmentWeight, cohesionWeight, separationWeight,
        obstacleAvoidanceWeight);

    for (int i = 0; i < DRONE_COUNT; i++) {
      drones.push_back(
          new Drone(m_world,
                    b2Vec2(rand() % static_cast<int>(screenWidth),
                           rand() % static_cast<int>(screenHeight)),
                    flockingBehaviour, viewRange, maxSpeed, maxForce));
    }
  }

  void DestroyDrones() {
    for (auto &drone : drones) {
      m_world->DestroyBody(drone->getBody());
    }
    drones.clear();
  }

  SwarmBehaviour *getSelectedBehaviour(int behaviourIndex) {
    switch (behaviourIndex) {
      case (0):
        // FlockingBehaviour
        return flockingBehaviour;
      case (1):
        // PheremoneBehaviour
        return pheremoneBehaviour;
      default:
        return nullptr;
    }
    return nullptr;
  }

  void SetBehaviour(int behaviourIndex) {
    SwarmBehaviour *newBehaviour = getSelectedBehaviour(behaviourIndex);
    for (auto &drone : drones) {
      drone->setBehaviour(newBehaviour);
    }
  }

  void UpdateDroneSettings() {
    // Update drone settings:
    for (auto &drone : drones) {
      drone->setMaxForce(maxForce);
      drone->setMaxSpeed(maxSpeed);
      drone->setViewRange(viewRange);
    }
  }

  void UpdateBehaviourSettings(int behaviourIndex) {
    SwarmBehaviour *newBehaviour;
    switch (behaviourIndex) {
      case (0):
        // FlockingBehaviour
        newBehaviour = new FlockingBehaviour(
            separationDistance, alignmentWeight, cohesionWeight,
            separationWeight, obstacleAvoidanceWeight);
        break;
      case (1):
        // PheremoneBehaviour
        newBehaviour = new PheremoneBehaviour(decayRate);
        break;
    }
    // Update the drones with the new behaviour
    for (auto &drone : drones) {
      drone->setBehaviour(newBehaviour);
    }
  }

  void UpdateUI() override {
    ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
    ImGui::SetNextWindowSize(ImVec2(285.0f, 285.0f));
    ImGui::Begin("Swarm Controls", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

    // Dropdown to select the behaviour
    if (ImGui::Combo("Behaviour", &currentBehaviour, behaviours,
                     IM_ARRAYSIZE(behaviours))) {
      SetBehaviour(currentBehaviour);
    }

    // Drone settings
    ImGui::Text("Drone Settings");
    bool speedChanged = ImGui::SliderFloat("maxSpeed", &maxSpeed, 0.0f, 50.0f);
    bool forceChanged = ImGui::SliderFloat("maxForce", &maxForce, 0.0f, 10.0f);
    bool viewChanged =
        ImGui::SliderFloat("viewRange", &viewRange, 0.0f, 100.0f);

    if (speedChanged || forceChanged || viewChanged) {
      UpdateDroneSettings();
    }

    // Display different behaviour settings depending on the behaviour selected
    ImGui::Text("Behaviour Settings");
    bool distance, alignment, cohesion, separation, avoidance, decay;
    switch (currentBehaviour) {
      case 0:
        // FlockingBehaviour
        distance = ImGui::SliderFloat("separationDistance", &separationDistance,
                                      0.0f, 100.0f);
        alignment =
            ImGui::SliderFloat("alignmentWeight", &alignmentWeight, 0.0f, 2.0f);
        cohesion =
            ImGui::SliderFloat("cohesionWeight", &cohesionWeight, 0.0f, 2.0f);
        separation = ImGui::SliderFloat("separationWeight", &separationWeight,
                                        0.0f, 2.0f);
        avoidance = ImGui::SliderFloat("obstacleAvoidanceWeight",
                                       &obstacleAvoidanceWeight, 0.0f, 2.0f);
        if (distance || alignment || cohesion || separation || avoidance) {
          UpdateBehaviourSettings(currentBehaviour);
        }
        break;
      case 1:
        // Pheremone Behaviour
        decay = ImGui::SliderFloat("decayRate", &decayRate, 0.0f, 1.0f);
        if (decay) {
          UpdateBehaviourSettings(currentBehaviour);
        }
        break;
    }

    if (ImGui::Button("Reset Simulation")) {
      DestroyDrones();
      CreateDrones();
    }

    ImGui::End();
  }

  static Test *Create() { return new DroneSwarmTest; }

  void Step(Settings &settings) override {
    Test::Step(settings);

    // Update Drone position
    for (auto &drone : drones) {
      drone->update(drones);
    }
  }
};

static int testIndex =
    RegisterTest("Swarm", "DroneSwarmTest", DroneSwarmTest::Create);
