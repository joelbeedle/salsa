
#include "core/test_queue.h"

namespace swarm {

std::vector<TestConfig> TestQueue::tests_;

void TestQueue::push(TestConfig test) { tests_.push_back(test); }

TestConfig TestQueue::pop() {
  if (tests_.empty()) {
    throw std::underflow_error("Cannot pop from an empty stack");
  }
  TestConfig test = tests_.front();
  tests_.erase(tests_.begin());
  return test;
}

TestConfig TestQueue::peek() {
  if (tests_.empty()) {
    throw std::underflow_error("Cannot peek from an empty stack");
  }
  return tests_.front();
}
}  // namespace swarm