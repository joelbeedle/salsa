// SwarmBehaviour.h
#pragma once
#include <vector>

class Drone;

class SwarmBehaviour {
 public:
  virtual ~SwarmBehaviour() = default;

  virtual void execute(std::vector<Drone *> &drones, Drone *currentDrone) = 0;
};