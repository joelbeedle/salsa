#include "core/test_stack.h"

#include <memory>

#include "behaviours/parameter.h"
#include "gtest/gtest.h"

using swarm::TestConfig;
using swarm::TestStack;
using swarm::behaviour::Parameter;

class TestStackTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Clear the stack before each test
    while (!TestStack::isEmpty()) {
      TestStack::pop();
    }
  }
};

TEST_F(TestStackTest, PushAndPopTest) {
  Parameter param(5.0, 1.0, 10.0);
  TestConfig::Parameters parameters;
  parameters.insert({"speed", &param});

  TestConfig testConfig1 = {"TestBehaviour1", parameters, nullptr, 100.0f,
                            100.0f,           5,          3,       120.0f};

  TestStack::push(testConfig1);
  EXPECT_EQ(1, TestStack::size());

  auto poppedConfig = TestStack::pop();
  EXPECT_EQ("TestBehaviour1", poppedConfig.behaviour_name);
  EXPECT_EQ(100.0f, poppedConfig.world_height);
  EXPECT_EQ(5, poppedConfig.num_drones);
}

TEST_F(TestStackTest, PopEmptyStack) {
  EXPECT_THROW(TestStack::pop(), std::exception);
}

TEST_F(TestStackTest, MaintainOrder) {
  TestConfig testConfig1 = {
      "Behaviour1",
      std::variant<TestConfig::Parameters, TestConfig::FloatParameters>(),
      nullptr,
      50.0f,
      50.0f,
      10,
      2,
      30.0f};
  TestConfig testConfig2 = {
      "Behaviour2",
      std::variant<TestConfig::Parameters, TestConfig::FloatParameters>(),
      nullptr,
      75.0f,
      75.0f,
      20,
      5,
      60.0f};

  TestStack::push(testConfig1);
  TestStack::push(testConfig2);

  EXPECT_EQ("Behaviour2", TestStack::pop().behaviour_name);
  EXPECT_EQ("Behaviour1", TestStack::pop().behaviour_name);
}