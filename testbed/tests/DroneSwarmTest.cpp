#include <box2d/box2d.h>
#include <stdio.h>

#include "Drone.h"
#include "FlockingBehaviour.h"
#include "imgui/imgui.h"
#include "test.h"

void createScreenBounds(b2World *world, float screenWidth, float screenHeight) {
  // Define the ground body.
  b2BodyDef groundBodyDef;
  groundBodyDef.position.Set(0.0f, 0.0f);

  // Call the body factory which allocates memory for the ground body
  // from a pool and creates the ground box shape (also from a pool).
  // The body is also added to the world.
  b2Body *groundBody = world->CreateBody(&groundBodyDef);

  // Define the ground box shape.
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
  FlockingBehaviour *flockingBehaviour;
  std::vector<Drone *> drones;
  const float screenWidth = 100.0f;
  const float screenHeight = 100.0f;

  // Floats for FlockingBehaviour
  float viewRange, separationDistance, alignmentWeight, cohesionWeight,
      separationWeight, obstacleAvoidanceWeight, maxSpeed, maxForce;

  DroneSwarmTest() {
    {
      // Create Screen Bounds
      createScreenBounds(m_world, screenWidth, screenHeight);
    }
    {
      // Initialise world and drones
      b2Vec2 gravity(0.0f, 0.0f);
      m_world->SetGravity(gravity);

      std::vector<b2Body *> obstacles;
      b2Body *currentBody = m_world->GetBodyList();
      while (currentBody) {
        obstacles.push_back(currentBody);
        currentBody = currentBody->GetNext();
      }
      for (auto &obstacle : obstacles) {
        printf("Obstacle: %f, %f\n", obstacle->GetPosition().x,
               obstacle->GetPosition().y);
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
          viewRange, separationDistance, alignmentWeight, cohesionWeight,
          separationWeight, obstacleAvoidanceWeight, maxSpeed, maxForce);

      for (int i = 0; i < 50; i++) {
        drones.push_back(new Drone(m_world, b2Vec2(rand() % 100, rand() % 100),
                                   flockingBehaviour));
      }
    }
  }

  void ResetBehaviour() {
    FlockingBehaviour *newFlockingBehaviour = new FlockingBehaviour(
        viewRange, separationDistance, alignmentWeight, cohesionWeight,
        separationWeight, obstacleAvoidanceWeight, maxSpeed, maxForce);
    for (auto &drone : drones) {
      drone->setBehaviour(newFlockingBehaviour);
    }
  }

  void UpdateUI() override {
    ImGui::SetNextWindowPos(ImVec2(10.0f, 100.0f));
    ImGui::SetNextWindowSize(ImVec2(210.0f, 285.0f));
    ImGui::Begin("Swarm Controls", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

    if (ImGui::Button("Reset Behaviour")) {
      ResetBehaviour();
    }

    ImGui::SliderFloat("viewRange", &viewRange, 0.0f, 100.0f);
    ImGui::SliderFloat("separationDistance", &separationDistance, 0.0f, 100.0f);
    ImGui::SliderFloat("maxSpeed", &maxSpeed, 0.0f, 50.0f);
    ImGui::SliderFloat("maxForce", &maxForce, 0.0f, 10.0f);
    ImGui::SliderFloat("alignmentWeight", &alignmentWeight, 0.0f, 2.0f);
    ImGui::SliderFloat("cohesionWeight", &cohesionWeight, 0.0f, 2.0f);
    ImGui::SliderFloat("separationWeight", &separationWeight, 0.0f, 2.0f);
    ImGui::SliderFloat("obstacleAvoidanceWeight", &obstacleAvoidanceWeight,
                       0.0f, 2.0f);

    if (ImGui::Button("Reset Simulation")) {
      DestroyDrones();
      CreateDrones();
    }

    ImGui::End();
  }

  void CreateDrones() {
    flockingBehaviour = new FlockingBehaviour(
        viewRange, separationDistance, alignmentWeight, cohesionWeight,
        separationWeight, obstacleAvoidanceWeight, maxSpeed, maxForce);

    for (int i = 0; i < 50; i++) {
      drones.push_back(new Drone(m_world, b2Vec2(rand() % 100, rand() % 100),
                                 flockingBehaviour));
    }
  }

  void DestroyDrones() {
    for (auto &drone : drones) {
      m_world->DestroyBody(drone->getBody());
    }
    drones.clear();
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
