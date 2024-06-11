#ifndef SWARM_SIM_CORE_TEST_COMPOSITE_H
#define SWARM_SIM_CORE_TEST_COMPOSITE_H

#include "core/sim.h"
#include "core/test_stack.h"

namespace swarm {

class TestComposite {
 private:
  std::vector<Sim *> sims_;

 public:
  TestComposite() = default;
  ~TestComposite() = default;

  void addSim(Sim *sim) { sims_.push_back(sim); }

  void runSims() {
    for (auto sim : sims_) {
      sim->init();
    }
  }
};

}  // namespace swarm

#endif  // SWARM_SIM_CORE_TEST_COMPOSITE_H