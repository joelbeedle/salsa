#include "salsa/entity/drone.h"

#include <box2d/box2d.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mock_behaviour.h"
#include "salsa/behaviours/behaviour.h"
#include "salsa/entity/drone_configuration.h"
#include "salsa/entity/drone_factory.h"

using ::testing::_;
// Test fixture for Drone tests
class DroneTest : public ::testing::Test {
 protected:
  b2World *world;
  MockBehaviour behaviour;
  salsa::DroneConfiguration *config;

  void SetUp() override {
    world = new b2World(b2Vec2(0.0f, 0.0f));
    MockBehaviour behaviour;
    config = new salsa::DroneConfiguration("test", 5.0f, 3.0f, 2.0f, 1.0f, 0.5f,
                                           1.0f, 10.0f);
    salsa::CollisionManager::registerType<salsa::Drone>({});
  }

  void TearDown() override { delete world; }
};

TEST_F(DroneTest, DroneInitialization) {
  salsa::Drone drone(world, b2Vec2(0, 0), behaviour, *config);
  EXPECT_FLOAT_EQ(drone.camera_view_range(), 5.0f);
  EXPECT_FLOAT_EQ(drone.max_speed(), 2.0f);
  EXPECT_NE(world, nullptr);
  EXPECT_EQ(world->GetBodyCount(), 1);
  EXPECT_NE(drone.body(), nullptr);
}

TEST_F(DroneTest, DroneFactoryInitialisation) {
  auto drone =
      salsa::DroneFactory::createDrone(world, b2Vec2(0, 0), behaviour, *config);
  EXPECT_FLOAT_EQ(drone->camera_view_range(), 5.0f);
  EXPECT_FLOAT_EQ(drone->max_speed(), 2.0f);
  EXPECT_NE(world, nullptr);
  EXPECT_EQ(world->GetBodyCount(), 1);
  EXPECT_NE(drone->body(), nullptr);
}

TEST_F(DroneTest, DroneBehaviourExecution) {
  salsa::Drone drone(world, b2Vec2(0, 0), behaviour, *config);
  EXPECT_CALL(behaviour, execute(_, _)).Times(1);
  drone.update({});
}

TEST_F(DroneTest, UpdateDroneBehaviour) {
  salsa::Drone drone(world, b2Vec2(0, 0), behaviour, *config);
  MockBehaviour newBehaviour;
  EXPECT_CALL(newBehaviour, execute(_, _)).Times(1);
  drone.behaviour() = &newBehaviour;
  drone.update({});
}

TEST_F(DroneTest, MultipleDroneBehaviourExecution) {
  std::vector<std::unique_ptr<salsa::Drone>> drones;
  int n = 10;
  for (int i = 0; i < n; i++) {
    drones.push_back(salsa::DroneFactory::createDrone(world, b2Vec2(i, i),
                                                      behaviour, *config));
  }
  EXPECT_CALL(behaviour, execute(_, _)).Times(n);
  for (auto &drone : drones) {
    drone->update(drones);
  }
}

TEST_F(DroneTest, UpdateSensorRange) {
  salsa::Drone drone(world, b2Vec2(0, 0), behaviour, *config);
  float newRange = 10.0f;
  drone.camera_view_range(newRange);
  drone.updateSensorRange();

  // Check if sensor range updated correctly
  auto fixture = drone.body()->GetFixtureList();
  while (fixture) {
    if (fixture->IsSensor()) {
      auto shape = static_cast<b2CircleShape *>(fixture->GetShape());
      EXPECT_FLOAT_EQ(shape->m_radius, newRange);
      break;
    }
    fixture = fixture->GetNext();
  }
}
