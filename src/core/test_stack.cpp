
#include "core/test_stack.h"

namespace swarm {

std::stack<TestConfig> TestStack::tests_;

void TestStack::push(TestConfig test) { tests_.push(test); }

TestConfig TestStack::pop() {
  TestConfig test = tests_.top();
  tests_.pop();
  return test;
}
}  // namespace swarm