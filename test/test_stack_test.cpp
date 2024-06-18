#include <memory>

#include "behaviours/parameter.h"
#include "core/test_queue.h"
#include "gtest/gtest.h"

using swarm::TestConfig;
using swarm::TestQueue;
using swarm::behaviour::Parameter;

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
      "TestBehaviour1", parameters, nullptr, nullptr, 100, 5, 3};

  TestQueue::push(testConfig1);
  EXPECT_EQ(1, TestQueue::size());

  auto poppedConfig = TestQueue::pop();
  EXPECT_EQ("TestBehaviour1", poppedConfig.behaviour_name);
  EXPECT_EQ(5, poppedConfig.num_drones);
}

TEST_F(TestQueueTest, PopEmptyStack) {
  EXPECT_THROW(TestQueue::pop(), std::exception);
}

TEST_F(TestQueueTest, MaintainOrder) {
  TestConfig testConfig1 = {
      "Behaviour1",
      std::variant<TestConfig::Parameters, TestConfig::FloatParameters>(),
      nullptr,
      nullptr,
      10,
      2,
      30.0f};
  TestConfig testConfig2 = {
      "Behaviour2",
      std::variant<TestConfig::Parameters, TestConfig::FloatParameters>(),
      nullptr,
      nullptr,
      20,
      5,
      60.0f};

  TestQueue::push(testConfig1);
  TestQueue::push(testConfig2);

  EXPECT_EQ("Behaviour2", TestQueue::pop().behaviour_name);
  EXPECT_EQ("Behaviour1", TestQueue::pop().behaviour_name);
}