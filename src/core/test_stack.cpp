
#include "core/test_stack.h"

namespace swarm {

void TestStack::pushTest(BehaviourWrapper *behaviour,
                         DroneConfiguration *config) {
  Test test = {behaviour, config};
  tests_.push(test);
}

}  // namespace swarm