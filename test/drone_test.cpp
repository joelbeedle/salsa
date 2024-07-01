#include "entity/drone.h"

#include <box2d/box2d.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "behaviours/behaviour.h"
#include "entity/drone_configuration.h"
#include "entity/drone_factory.h"
#include "mock_behaviour.h"

using ::testing::_;
// Test fixture for Drone tests
class DroneTest : public ::testing::Test {
 protected:
  b2World *world;
  MockBehaviour behaviour;
  swarm::DroneConfiguration *config;

  void SetUp() override {
    world = new b2World(b2Vec2(0.0f, 0.0f));
    MockBehaviour behaviour;
    config = new swarm::DroneConfiguration("test", 5.0f, 3.0f, 2.0f, 1.0f, 0.5f,
                                           1.0f, 10.0f);
  }

  void TearDown() override { delete world; }
};

TEST_F(DroneTest, DroneInitialization) {
  swarm::Drone drone(world, b2Vec2(0, 0), behaviour, *config);
  EXPECT_FLOAT_EQ(drone.getViewRange(), 5.0f);
  EXPECT_FLOAT_EQ(drone.getMaxSpeed(), 2.0f);
  EXPECT_NE(world, nullptr);
  EXPECT_EQ(world->GetBodyCount(), 1);
  EXPECT_NE(drone.getBody(), nullptr);
}

TEST_F(DroneTest, DroneFactoryInitialisation) {
  auto drone =
      swarm::DroneFactory::createDrone(world, b2Vec2(0, 0), behaviour, *config);
  EXPECT_FLOAT_EQ(drone->getViewRange(), 5.0f);
  EXPECT_FLOAT_EQ(drone->getMaxSpeed(), 2.0f);
  EXPECT_NE(world, nullptr);
  EXPECT_EQ(world->GetBodyCount(), 1);
  EXPECT_NE(drone->getBody(), nullptr);
}

TEST_F(DroneTest, DroneBehaviourExecution) {
  swarm::Drone drone(world, b2Vec2(0, 0), behaviour, *config);
  EXPECT_CALL(behaviour, execute(_, _)).Times(1);
  drone.update({});
}

TEST_F(DroneTest, UpdateDroneBehaviour) {
  swarm::Drone drone(world, b2Vec2(0, 0), behaviour, *config);
  MockBehaviour newBehaviour;
  EXPECT_CALL(newBehaviour, execute(_, _)).Times(1);
  drone.setBehaviour(newBehaviour);
  drone.update({});
}

TEST_F(DroneTest, MultipleDroneBehaviourExecution) {
  std::vector<std::unique_ptr<swarm::Drone>> drones;
  int n = 10;
  for (int i = 0; i < n; i++) {
    drones.push_back(swarm::DroneFactory::createDrone(world, b2Vec2(i, i),
                                                      behaviour, *config));
  }
  EXPECT_CALL(behaviour, execute(_, _)).Times(n);
  for (auto &drone : drones) {
    drone->update(drones);
  }
}

TEST_F(DroneTest, UpdateSensorRange) {
  swarm::Drone drone(world, b2Vec2(0, 0), behaviour, *config);
  float newRange = 10.0f;
  drone.setViewRange(newRange);
  drone.updateSensorRange();

  // Check if sensor range updated correctly
  auto fixture = drone.getBody()->GetFixtureList();
  while (fixture) {
    if (fixture->IsSensor()) {
      auto shape = static_cast<b2CircleShape *>(fixture->GetShape());
      EXPECT_FLOAT_EQ(shape->m_radius, newRange);
      break;
    }
    fixture = fixture->GetNext();
  }
}
