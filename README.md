# swarm-sim

Swarm Intelligence Algorithms in Box2D

`testbed` folder code is adapted from the [Box2D testbed](https://github.com/erincatto/box2d), with modifications. All code attributed to Erin Catto has their license at the top.

## Getting Started

The directory is split between a library, `libswarm-sim`, and a binary testbed `swarm-testbed`.

`libswarm-sim` contains the library, and `swarm-testbed` an easy to use setup of it.

### Example of using the Testbed

User defined algorithms should be placed in the `testbed/algorithms` folder. An algorithm has to extend the `swarm::Behaviour` class, and implement the virtual function `execute()`.

Once an algorithm is written, the user needs to set up the simulation. This is done by editing the `main` file for the `testbed`, in `testbed/main.cpp`.

The user creates an instance of their algorithm, for example such as:

```cpp
#include <swarm_simulator.h>
// Create Flocking behaviour and assign initial parameters
auto flocking = std::make_unique<Flocking>(250.0, 1.6, 1.0, 3.0, 3.0);

// Create a drone configuration with initial parameters
auto *small_drone = new swarm::DroneConfiguration(
    25.0f, 50.0f, 10.0f, 0.3f, 1.0f, 1.5f, 4000.0f);

// Set up the listener to listen for collisions
auto listener = std::make_shared<swarm::BaseContactListener>();

// Add a collision handler between Drone and Target types, when this collision is detected, userHandlingFunction is called to handle the collision.
listener.addColisionHandler(typeid(swarm::Drone), typeid(swarm::Target), userHandlingFunction)

// Set up test environment
SwarmTest::SetHeight(BORDER_HEIGHT);
SwarmTest::SetWidth(BORDER_WIDTH);

// Set the simulation contact listener to be our user defined listener
SwarmTest::SetContactListener(*listener);

// Register behaviors in the simulation
SwarmTest::AddBehaviour("Flocking", std::move(flock));
SwarmTest::SetDroneConfiguration(small_drone);

// Register a type of target for the drones to detect, here `Tree` is a user defined class.
SwarmTest::AddTargetType(typeid(Tree));
SwarmTest::SetNumDrones(DRONE_COUNT);
SwarmTest::SetNumTargets(typeid(Tree), TREE_COUNT);

// TODO: IMPLEMENT and redesign?
// Use a test stack, and point to a file that defines it, in the format:
// {Behaviour Name},{Behaviour Parameters},{Drone Count},{DroneConfig},{Target Type},{Target Count},{Max time}
SwarmTest::SetTestStack("path/to/test/stack.txt");

// Alternatively, use a test stack, and initialise it here:
auto *test_stack = std::make_unique<swarm::TestStack>();
test_stack.addTest()
        .setNumDrones(50)
        .setDroneConfig(myDroneConfig)
        .setBehaviour("Flocking");
        .setParameters(250.0, 1.6, 1.0, 3.0, 3.0)
        .setNumTargets(typeid(Tree), 10000);
        .setMaxTime(20000);

// Or, use a smart test stack in order to permute through some parameters

// Begin the simulation
SwarmTest::Run();
```

## Installation

### Dependencies

- CMake
- A C/C++ compiler (gcc, clang, cl)
- Box2D (packaged with directory)
- OpenGL and GLFW

### Steps

1. Clone the github directory using `git clone --recursive https://github.com/joelbeedle/swarm-sim.git`
   - Note: the `--recursive` tag is important, as this also clones the Box2D submodule
2. Set up the Box2D submodule using `git submodule update --init --recursive`
3. Run the following commands:
   - `mkdir build`
   - `cd build`
   - `cmake .`
4. The `testbed` binary will be found in `build/testbed`, the library `.a` file in `build/src`, and the docs in `build/docs`.
5. To run the testbed, use `./testbed`.

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
