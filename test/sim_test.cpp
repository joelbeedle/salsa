#include "core/sim.h"

#include <memory>

#include "behaviours/registry.h"
#include "core/test_stack.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mock_behaviour.h"
#include "mock_drone.h"
#include "utils/drone_configuration.h"

using swarm::DroneConfiguration;
using swarm::Sim;
using swarm::behaviour::Registry;

class SimTest : public ::testing::Test {
 protected:
  b2World world{b2Vec2(0.0f, 0.0f)};  // Zero gravity world for testing
  std::unique_ptr<Sim> sim;
  DroneConfiguration* config;
  MockBehaviour behaviour;
  const std::string behaviour_name = "TestBehaviour";

  void SetUp() override {
    auto mockBehaviour = std::make_unique<MockBehaviour>();
    swarm::behaviour::Registry::getInstance().add(behaviour_name,
                                                  std::move(mockBehaviour));

    config = new DroneConfiguration(5.0f, 3.0f, 2.0f, 1.0f, 0.5f, 1.0f, 10.0f);
    sim = std::make_unique<Sim>(&world, 5, 3, config, 100.0f, 100.0f, 120.0f);
    sim->setCurrentBehaviour(behaviour_name);
    sim->applyCurrentBehaviour();
    std::cout << sim->getDroneCount() << std::endl;
  }

  void TearDown() override {
    delete config;
    swarm::behaviour::Registry::getInstance().remove(behaviour_name);
  }
};

TEST_F(SimTest, ConstructorTest) { EXPECT_EQ(5, sim->getDroneCount()); }

TEST_F(SimTest, CorrectBehaviourTest) {
  EXPECT_EQ(behaviour_name, sim->getBehaviourName());
  sim->applyCurrentBehaviour();
  auto& drones = sim->getDrones();
  for (const auto& drone : drones) {
    auto behaviour = Registry::getInstance().getBehaviour(behaviour_name);
    auto drone_behaviour = drone->getBehaviour();
    EXPECT_EQ(behaviour, drone_behaviour);
  }
}

TEST_F(SimTest, SetDroneConfigurationTest) {
  DroneConfiguration newConfig =
      DroneConfiguration(10.0f, 5.0f, 3.0f, 2.0f, 1.0f, 0.5f, 20.0f);
  sim->setDroneConfiguration(&newConfig);
  EXPECT_EQ(&newConfig, sim->getDroneConfiguration());
}

TEST_F(SimTest, UpdateTest) { ASSERT_NO_THROW(sim->update()); }

TEST_F(SimTest, ResetTest) {
  sim->reset();
  EXPECT_EQ(5, sim->getDroneCount());
}