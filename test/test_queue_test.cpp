#include "salsa/core/test_queue.h"

#include <memory>

#include "gtest/gtest.h"
#include "salsa/behaviours/parameter.h"

using salsa::TestConfig;
using salsa::TestQueue;
using salsa::behaviour::Parameter;

class TestQueueTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Clear the stack before each test
    while (!TestQueue::isEmpty()) {
      TestQueue::pop();
    }
  }
};

TEST_F(TestQueueTest, PushAndPopTest) {
  Parameter param(5.0, 1.0, 10.0);
  TestConfig::Parameters parameters;
  parameters.insert({"speed", &param});

  TestConfig testConfig1 = {
      "Behaviour1",
      std::variant<TestConfig::Parameters, TestConfig::FloatParameters>(),
      nullptr,
      "",
      100,
      0,
      1200.0f,
  };
  TestQueue::push(testConfig1);
  EXPECT_EQ(1, TestQueue::size());

  auto poppedConfig = TestQueue::pop();
  EXPECT_EQ(100, poppedConfig.num_drones);
  EXPECT_EQ("Behaviour1", poppedConfig.behaviour_name);
}

TEST_F(TestQueueTest, PopEmptyStack) {
  EXPECT_THROW(TestQueue::pop(), std::exception);
}

TEST_F(TestQueueTest, MaintainOrder) {
  TestConfig config1 = {
      "Behaviour1",
      std::variant<TestConfig::Parameters, TestConfig::FloatParameters>(),
      nullptr,
      "",
      100,
      0,
      1200.0f,
  };
  TestConfig config2 = {
      "Behaviour2",
      std::variant<TestConfig::Parameters, TestConfig::FloatParameters>(),
      nullptr,
      "",
      100,
      0,
      1200.0f,
  };
  TestQueue::push(config1);
  TestQueue::push(config2);

  EXPECT_EQ("Behaviour1", TestQueue::pop().behaviour_name);
  EXPECT_EQ("Behaviour2", TestQueue::pop().behaviour_name);
}