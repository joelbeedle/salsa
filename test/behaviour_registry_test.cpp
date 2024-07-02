#include <box2d/box2d.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "mock_behaviour.h"
#include "salsa/behaviours/registry.h"

using swarm::Behaviour;
using swarm::behaviour::Registry;

class RegistryTest : public ::testing::Test {
 protected:
  Registry& registry =
      Registry::getInstance();  // Singleton instance for all tests

  void SetUp() override {
    // Ensure the registry is clean before each test
    auto names = registry.getBehaviourNames();
    for (const auto& name : names) {
      registry.remove(name);
    }
  }
};

TEST_F(RegistryTest, SingletonInstance) {
  Registry& registry1 = Registry::getInstance();
  Registry& registry2 = Registry::getInstance();
  EXPECT_EQ(&registry1, &registry2);  // Both instances should be the same
}

TEST_F(RegistryTest, AddAndGetBehaviour) {
  auto mockBehaviour = std::make_unique<MockBehaviour>();
  auto* rawPointer = mockBehaviour.get();
  registry.add("TestBehaviour", std::move(mockBehaviour));

  Behaviour* retrievedBehaviour = registry.getBehaviour("TestBehaviour");
  EXPECT_EQ(retrievedBehaviour, rawPointer);
}

TEST_F(RegistryTest, RetrieveNonExistentBehaviour) {
  Behaviour* retrievedBehaviour = registry.getBehaviour("NonExistent");
  EXPECT_EQ(retrievedBehaviour, nullptr);
}

TEST_F(RegistryTest, AddDuplicateBehaviour) {
  auto mockBehaviour = std::make_unique<MockBehaviour>();
  registry.add("TestBehaviour", std::move(mockBehaviour));

  auto mockBehaviour2 = std::make_unique<MockBehaviour>();
  registry.add("TestBehaviour", std::move(mockBehaviour2));

  Behaviour* retrievedBehaviour = registry.getBehaviour("TestBehaviour");
  EXPECT_NE(retrievedBehaviour, nullptr);
}

TEST_F(RegistryTest, RemoveBehaviour) {
  auto mockBehaviour = std::make_unique<MockBehaviour>();
  registry.add("TestBehaviour", std::move(mockBehaviour));

  registry.remove("TestBehaviour");

  Behaviour* retrievedBehaviour = registry.getBehaviour("TestBehaviour");
  EXPECT_EQ(retrievedBehaviour, nullptr);
}

TEST_F(RegistryTest, RemoveNonExistentBehaviour) {
  bool result = registry.remove("NonExistent");
  EXPECT_FALSE(result);
}

TEST_F(RegistryTest, ListBehaviourNames) {
  registry.add("Behaviour1", std::make_unique<MockBehaviour>());
  registry.add("Behaviour2", std::make_unique<MockBehaviour>());
  registry.add("Behaviour3", std::make_unique<MockBehaviour>());

  auto names = registry.getBehaviourNames();
  EXPECT_EQ(names.size(), 3);
  EXPECT_NE(std::find(names.begin(), names.end(), "Behaviour1"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "Behaviour2"), names.end());
  EXPECT_NE(std::find(names.begin(), names.end(), "Behaviour3"), names.end());
}