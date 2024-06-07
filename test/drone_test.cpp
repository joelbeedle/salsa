#include "drones/drone.h"

#include <box2d/box2d.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "behaviours/behaviour.h"
#include "drones/drone_factory.h"
#include "utils/drone_configuration.h"
using ::testing::_;

class MockBehaviour : public swarm_sim::Behaviour {
 public:
  MOCK_METHOD(void, execute,
              (const std::vector<std::unique_ptr<swarm_sim::Drone>>& drones,
               swarm_sim::Drone& currentDrone),
              (override));

  MOCK_METHOD((std::unordered_map<std::string, swarm_sim::ParameterDefinition>),
              getParameters, (), (override));
  MOCK_METHOD(void, clean,
              (const std::vector<std::unique_ptr<swarm_sim::Drone>>& drones),
              (override));
};

// Test fixture for Drone tests
class DroneTest : public ::testing::Test {
 protected:
  b2World* world;
  MockBehaviour behaviour;
  swarm_sim::DroneConfiguration* config;

  void SetUp() override {
    world = new b2World(b2Vec2(0.0f, 0.0f));
    MockBehaviour behaviour;
    config = new swarm_sim::DroneConfiguration(5.0f, 3.0f, 2.0f, 1.0f, 0.5f,
                                               1.0f, 10.0f);
  }

  void TearDown() override { delete world; }
};

TEST_F(DroneTest, DroneInitialization) {
  swarm_sim::Drone drone(world, b2Vec2(0, 0), behaviour, *config);
  EXPECT_FLOAT_EQ(drone.getViewRange(), 5.0f);
  EXPECT_FLOAT_EQ(drone.getMaxSpeed(), 2.0f);
  EXPECT_NE(drone.getBody(), nullptr);
}

TEST_F(DroneTest, DroneBehaviourExecution) {
  swarm_sim::Drone drone(world, b2Vec2(0, 0), behaviour, *config);
  std::vector<std::unique_ptr<swarm_sim::Drone>> drones;
  drones.push_back(std::make_unique<swarm_sim::Drone>(drone));

  EXPECT_CALL(behaviour, execute(_, _)).Times(1);

  for (auto& drone : drones) {
    GTEST_LOG_(INFO) << "Testing drone";
    drone->update(drones);
  }
}

TEST_F(DroneTest, UpdateSensorRange) {
  swarm_sim::Drone drone(world, b2Vec2(0, 0), behaviour, *config);
  float newRange = 10.0f;
  drone.setViewRange(newRange);
  drone.updateSensorRange();

  // Check if sensor range updated correctly
  auto fixture = drone.getBody()->GetFixtureList();
  while (fixture) {
    if (fixture->IsSensor()) {
      auto shape = static_cast<b2CircleShape*>(fixture->GetShape());
      EXPECT_FLOAT_EQ(shape->m_radius, newRange);
      break;
    }
    fixture = fixture->GetNext();
  }
}
