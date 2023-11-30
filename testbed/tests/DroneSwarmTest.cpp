#include <box2d/box2d.h>
#include <stdio.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include "Drone.h"
#include "FlockingBehaviour.h"
#include "PheremoneBehaviour.h"
#include "imgui/imgui.h"
#include "test.h"

#define DRONE_COUNT 50
#define BORDER_WIDTH 100.0f
#define BORDER_HEIGHT 100.0f

// TODO: add numBehaviours at end to ensure it is updated throughout
enum class BehaviourType { Flocking, Pheremone };

struct BehaviourTypeInfo {
  BehaviourType type;
  std::string displayName;
};

class BehaviourFactory {
 public:
  using BehaviourParams = std::variant<FlockingParameters, PheremoneParameters>;

  static SwarmBehaviour *createBehaviour(BehaviourType type,
                                         BehaviourParams params) {
    switch (type) {
      case BehaviourType::Flocking:
        return new FlockingBehaviour(
            extractParameters<FlockingParameters>(params));
      case BehaviourType::Pheremone:
        return new PheremoneBehaviour(
            extractParameters<PheremoneParameters>(params));
      default:
        throw std::runtime_error("Unknown behaviour type");
    }
  }

 private:
  template <typename T>
  static T extractParameters(const BehaviourParams &params) {
    if (auto p = std::get_if<T>(&params)) {
      return *p;
    } else {
      throw std::runtime_error("Incorrect parameters for behaviour");
    }
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

  // Parameters
  FlockingParameters flockingParams = {50.0f, 1.0f, 1.0f, 1.0f};
  PheremoneParameters pheremoneParams = {0.1f};

  // Drone settings
  std::vector<Drone *> drones;

  float viewRange = 20.0f;
  float maxSpeed = 10.0f;
  float maxForce = 0.3f;

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
      initDefaultParameters();

      // Set initial behaviour
      currentBehaviourType = BehaviourType::Flocking;
      currentBehaviourIndex = 0;

      // Set behaviour to equal our current behaviour
      UpdateBehaviour();
      // Create our drones with our selected behaviour
      createDrones(behaviour);
    }
  }

  void initWorld() {
    createBounds(m_world);
    b2Vec2 gravity(0.0f, 0.0f);
    m_world->SetGravity(gravity);
  }

  void initDefaultParameters() {
    viewRange = 20.0f;
    maxSpeed = 10.0f;
    maxForce = 0.3f;

    flockingParams = {50.0f, 1.0f, 1.0f, 1.0f};
    pheremoneParams = {0.1f};
  }

  void createDrones(SwarmBehaviour *b) {
    for (int i = 0; i < DRONE_COUNT; i++) {
      drones.push_back(
          new Drone(m_world,
                    b2Vec2(rand() % static_cast<int>(BORDER_WIDTH),
                           rand() % static_cast<int>(BORDER_HEIGHT)),
                    behaviour));
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
          UpdateBehaviour();  // Assuming this function handles the logic for
                              // updating behaviour
          SetBehaviour();  // Assuming this function applies the new behaviour
        }
        if (isSelected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
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
    switch (currentBehaviourIndex) {
      case 0:
        // FlockingBehaviour
        distance = ImGui::SliderFloat("separationDistance",
                                      &flockingParams.separationDistance, 0.0f,
                                      100.0f);
        alignment = ImGui::SliderFloat(
            "alignmentWeight", &flockingParams.alignmentWeight, 0.0f, 2.0f);
        cohesion = ImGui::SliderFloat(
            "cohesionWeight", &flockingParams.cohesionWeight, 0.0f, 2.0f);
        separation = ImGui::SliderFloat(
            "separationWeight", &flockingParams.separationWeight, 0.0f, 2.0f);
        avoidance = ImGui::SliderFloat("obstacleAvoidanceWeight",
                                       &flockingParams.obstacleAvoidanceWeight,
                                       0.0f, 2.0f);
        if (distance || alignment || cohesion || separation || avoidance) {
          UpdateBehaviour();
          SetBehaviour();
        }
        break;
      case 1:
        // Pheremone Behaviour
        decay = ImGui::SliderFloat("decayRate", &pheremoneParams.decayRate,
                                   0.0f, 1.0f);
        if (decay) {
          UpdateBehaviour();
          SetBehaviour();
        }
        break;
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

    // Update Drone position
    for (auto &drone : drones) {
      drone->update(drones);
    }
  }
};

static int testIndex =
    RegisterTest("Swarm", "DroneSwarmTest", DroneSwarmTest::Create);
