# swarm-sim

Swarm Algorithm Simulation library and Testbed, written in C++.

`testbed` folder code is originally adapted from the [Box2D testbed](https://github.com/erincatto/box2d).

## Getting Started

### Installation

#### Prerequisites

- CMake 3.14+
- A C++17 compatible compiler (gcc, clang, cl)
- Git
- OpenGL and GLFW
- Optional: Doxygen

#### Steps

1. Clone the github directory using `git clone --recursive https://github.com/joelbeedle/swarm-sim.git`
   - Note: the `--recursive` tag is important, as this also clones the Box2D submodule
2. Set up the Box2D submodule using `git submodule update --init --recursive`
3. To configure, inside the root `swarm-sim` folder, run `cmake -S . -B build`
   - If you have Ninja installed, you can run: `cmake -S . -B build -GNinja`
4. To build, run `cmake --build build`
   - To run tests, `cmake --build build --target test`
5. To build docs (this requires Doxygen, docs can be found in `build/docs/html`), run `cmake --build --target docs`
6. To run the testbed, then navigate to `build/testbed` and run `./testbed`.

The directory is split between a library, `libswarm-sim`, and a binary testbed `swarm-testbed`.

`libswarm-sim` contains the library, and `swarm-testbed` an easy to use setup of it.

### Example of using the Testbed

User defined algorithms should be placed in the `testbed/algorithms` folder. An algorithm has to extend the `swarm::Behaviour` class, and implement the virtual function `execute()`.

Once an algorithm is written, the user needs to set up the simulation. This is done by editing the `main` file for the `testbed`, in `testbed/main.cpp`.

The user creates an instance of their algorithm, for example such as:

```cpp
#include <testbed.h>
// Create Flocking behaviour and assign initial parameters
auto behaviour_parameters = std::make_unique<CustomBehaviour>(250.0, 1.6, 1.0, 3.0, 3.0);

// Create a drone configuration with initial parameters
auto *drone_config = new swarm::DroneConfiguration(
    25.0f, 50.0f, 10.0f, 0.3f, 1.0f, 1.5f, 4000.0f);

// Set up the listener to listen for collisions
auto listener = std::make_shared<swarm::BaseContactListener>();

// Add a collision handler between Drone and Target types, when this collision is detected, userHandlingFunction is called to handle the collision.
listener.addColisionHandler(typeid(swarm::Drone), typeid(swarm::Target), userHandlingFunction)

// Set up test environment
auto map = map::load("map.json");
auto num_drones = 100;
auto num_targets = 1000;
auto time_limit = 1200.0;

swarm::TestConfig test = {
   "Custom Behaviour",
   behaviour_parameters,
   drone_config,
   map,
   num_drones,
   num_targets,
   time_limit,
};

// Create queue and add test to it
swarm::TestQueue queue;
queue.push(test);

// Begin the simulation
testbed::run();
```

## Installation

## Design

The user facing design can be split into three distinct components:

1. Algorithms
   - The user designed algorithms to run in the simulation
2. Data
   - The data that the user wants the simulation to output.
   - Features common to investigating Swarm Algorithms can be selected (e.g. avg. Agent distance), and custom data to be added can be defined by the user.
3. Test Stack
   - The algorithm, and simulation parameters to run a test for.
   - Can be ran headless, or with a GUI.
