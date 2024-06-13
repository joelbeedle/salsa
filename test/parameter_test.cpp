#include "behaviours/parameter.h"

#include <iostream>
#include <sstream>

#include "gtest/gtest.h"

using swarm::behaviour::Parameter;

class ParameterTest : public ::testing::Test {
 protected:
  // Redirect std::cerr to capture output messages
  std::stringstream buffer;
  std::streambuf* old;

  void SetUp() override { old = std::cerr.rdbuf(buffer.rdbuf()); }

  void TearDown() override { std::cerr.rdbuf(old); }
};

TEST_F(ParameterTest, ConstructorValidInitialValue) {
  ASSERT_NO_THROW(Parameter p(0.5, 0.0, 1.0));
  Parameter p(0.5, 0.0, 1.0);
  EXPECT_EQ(0.5f, p.value());
}

TEST_F(ParameterTest, ConstructorInvalidInitialValue) {
  EXPECT_THROW(Parameter p(1.5, 0.0, 1.0), std::out_of_range);
}

TEST_F(ParameterTest, AssignmentWithinBounds) {
  Parameter p(0.5, 0.0, 1.0);
  p = 0.8;
  EXPECT_EQ(0.8f, p.value());
}

TEST_F(ParameterTest, AssignmentOutOfBounds) {
  Parameter p(0.5, 0.0, 1.0);
  p = 1.5;
  EXPECT_EQ(0.5f, p.value());
  EXPECT_EQ("Assignment value out of bounds!\n", buffer.str());
}

TEST_F(ParameterTest, ImplicitConversionToFloat) {
  Parameter p(0.5, 0.0, 1.0);
  float value = p;
  EXPECT_EQ(0.5f, value);
}

TEST_F(ParameterTest, AccessorsAndMutators) {
  Parameter p(0.5, 0.0, 1.0);
  EXPECT_EQ(0.5f, p.value());
  p.value() = 0.7f;
  EXPECT_EQ(0.7f, p.value());

  EXPECT_EQ(0.0f, p.min_value());
  EXPECT_EQ(1.0f, p.max_value());

  p.min_value() = 0.2f;
  p.max_value() = 0.8f;

  EXPECT_EQ(0.2f, p.min_value());
  EXPECT_EQ(0.8f, p.max_value());
}
