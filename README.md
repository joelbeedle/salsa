# swarm-sim

Swarm Algorithm Simulation library and Testbed, written in C++.

`testbed` folder code is originally adapted from the [Box2D testbed](https://github.com/erincatto/box2d).

## Getting Started
There is a virtual image with Ubuntu available [here](). 

The `swarm-sim` files were installed on the disk image using the following instructions, and can be found at `~/swarm-sim`. You can use the CMake commands in the Installation section to re-configure and re-build this when modifying the code.

There is also a Docker container available. It opens a localhost noVNC web app with `swarm-sim` pre-installed.

### Using the Docker container

Clone the repository
```bash
git clone --recursive https://github.com/joelbeedle/swarm-sim.git
git submodule update --init --recursive
```

Then, build the docker image
```bash
docker build -t swarm-test/testbed .
```

Once it has built, run the container using
```bash
docker run --shm-size=256m -it -p 5901:5901 -e VNC_PASSWD=123456 swarm-test/testbed
```

We can then connect to the virtual machine by going to `http://localhost:5901` and enter the password `123456`.

Then, right click, select `Applications > Terminal > Bash` and you will find yourself already in a terminal in the repository.

The testbed application can be found at `./build/testbed/`

### Installation
`swarm-sim` was designed with to be cros-platform.

#### Requirements
- CMake **3.14+**
- A **C++17** compatible compiler (gcc, clang, cl)
- Git
- OpenGL
- Optional: [Doxygen](https://github.com/doxygen/doxygen), [Ninja](https://github.com/ninja-build/ninja)

Requirements in bold are **essential**.

`swarm-sim` uses the following packages directly:
- [Box2D](https://github.com/erincatto/box2d)
- [GLAD](https://github.com/Dav1dde/glad) and [GLFW]()
- [spdlog](https://github.com/gabime/spdlog)
- [nlohmann_json](https://github.com/nlohmann/json)
- [CLI11](https://github.com/CLIUtils/CLI11)
- [ImGui](https://github.com/ocornut/imgui)
- [googletest](https://github.com/google/googletest)
  
These requirements are all either managed as git submodules, or are fetched and installed into `build/_deps` when configuring using CMake's [FetchContent](https://cmake.org/cmake/help/latest/module/FetchContent.html) module automatically, so you shouldn't have to do anything to set them up.

It may also be neccessary to install some extra dependencies on Linux (or just do `sudo apt-get install -y build-essentials ubuntu-desktop`). If you get build errors, install the missing packages. 

#### Steps
1. Clone the repository and all submodules:
   ```bash
   git clone --recursive https://github.com/joelbeedle/swarm-sim.git
   ```
   - Note: the `--recursive` tag is important, as this also clones the submodules, which are needed.
2. Initialise and update the submodules:
   ```bash
   git submodule update --init --recursive
   ```
3. Configure the project with CMake:
   ```bash
   cd swarm-sim
   cmake -S . -B build
   ```
    - If you have Ninja installed, for a faster build time:
   ```bash
   cmake -S . -B build -GNinja
   ```

4. Build the project:
   ```bash
   cmake --build build
   ```
5. Optionally, build documentation (requires Doxygen):
   ```bash
   cmake --build build --target docs
   ```
   - Documentation can then be found in `build/docs/html/index.html`
There are also some common build configurations found in `CMakePresents.json`.

### Running the Testbed
Now that the build is complete, to run the testbed:

1. Navigate to the `build/testbed` directory and execute `./testbed`. The testbed can also be ran headless in it's Simulator Queue mode using `./testbed --headless`. Use `./testbed --help` for a full list of commands.

The testbed, when opened, presents the main menu. From here, we can access the following features:
- Simulators
   - Queue Mode
   - Sandbox Mode
- Map Creator

It comes prepackaged with a few swarm algorithms, a drone configuration, and `Tree`s as targets.

## Extensibility
Users can extend the functionality of the testbed. They can add custom swarm algorithms, targets for the drones to find, custom contact listeners and contact managers, as well as custom drone configurations. Users can create their own maps for the simulations and could set new custom data to log. It is also possible to add custom testbed modes, similar to the current `Queue` and `Sandbox` modes.

For more information on extending the various aspects of the testbed, expand a section below.

<details>
  
<summary>Adding new Swarm Algorithms</summary>

### Add a Custom Swarm Algorithm
To add a custom swarm algorithm:
Create a new `.cpp` file inside `testbed/behaviors`, and name it, etc `my_alg.cpp`.

Inside this file, import `<core/simulation.h>` to import all library headers.

Create a class that extends the `Behaviour` class:
  ```cpp
  #include <core/simulation.h>

  class MyAlg : public Behaviour {};
  ```
Define any parameters as a behaviour::Parameter
   ```cpp
   class MyAlg : public swarm::Behaviour {
     private:
      behaviour::Parameter param_1_;
      behaviour::Parameter param_2_;
      behaviour::Parameter param_3_;
   };
   ```
Implement a constructor that takes as arguments the behaviour parameters, and pass these into the defined `behaviour::Parameter`s in the constructor initializer list.

Then, create a key value pair in `parameters_`, a map of names to parameters, using as a key the display name, and as value a reference to the internal `behaviour::Parameter`. 
  ```cpp
   class MyAlg : public swarm::Behaviour {
     private:
      behaviour::Parameter param_1_;
      behaviour::Parameter param_2_;
     public:
      // 1. Implement constructor, pass values in constructor initializer list.
      MyAlg(float param_1, float param_2) {
        // 2. Create key, value pair in parameters_ map.
        // parameters_ is defined in Behaviour and is a unordered_map<Parameter*>
        parameters_["Parameter 1"] = &param_1_;
        parameters_["Parameter 2"] = &param_2_;
      }   

  ```
Implement `execute()`, returning an acceleration vector for that drone.
```cpp
// <snip>
b2Vec2 execute(const std::vector<std::unique_ptr<Drone>> &drones,
            Drone &currentDrone) override {
   // Insert custom behaviour code here
   return drone_acceleration;
}
```
After the class definition, create an instance and register it with the behaviour registry with its initial parameters.
```cpp
auto alg = behaviour::Registry::getInstance().add(
    "My Alg",
    std::make_unique<MyAlg>(1.0, 3421.3));
```

This behaviour will now appear in the testbed, and can be set in a TestConfig using the behaviour namme specified, e.g. `My Alg`

</details>

<details>
  
<summary>Adding new Targets</summary>

### Add a custom Target
Create a `.cpp` and `.h` file in `testbed/targets`, and name it, etc. `my_target.h`.

In the header file, include `<core/simulation.h>` and create a class that extends `swarm::Target`
   ```cpp
   class MyTarget : public Target {};
   ```
In its constructor, a target **must** have the first 3 parameters as `b2World* world, const b2Vec2& position, int id`, but any other parameters are user definable.
   ```cpp
   MyTarget(b2World* world, const b2Vec2 &position, int id, bool visible, float radius, int arg) {} 
   ```
A target must follow this pattern for its body definitions:
   ```cpp
     // Shape can be anything
     b2CircleShape shape;
     shape.m_radius = radius;

     // Targets have a fixture that is a sensor
     b2FixtureDef fixtureDef;
     fixtureDef.shape = &shape;
     fixtureDef.isSensor = true;

     // We register this type with the collision manager, get a config back
     //  and set the filter category and mask the received settings.
     auto config = CollisionManager::getCollisionConfig<Tree>();
     fixtureDef.filter.categoryBits = config.categoryBits;
     fixtureDef.filter.maskBits = config.maskBits;

     // We then create a swarm::UserData object. This is metadata and how collision callbacks are managed.
     UserData *my_data = new UserData();
     // Set the userData's object argument to point to this instance
     my_data->object = this;
     // Set the userData of the fixtureDef as a pointer to our swarm::UserData object and create fixture
     fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(my_data);
     body_->CreateFixture(&fixtureDef);
   ```
We don't need to worry about creating the body, as this is done through the `Entity` class that `Target` inherits from. This is done to simplify creation and abstract the physics engine interface from the user.
Once the target is fully defined, we register it with the `TargetFactory` and the collision manager, in our `main.cpp` entrypoint
   ```cpp
   // main.cpp
   // If we want drones to detect this target type and the target type to also detect the trees, define both.
   // If not, `registerType` should take < The thing you want to do the detecting > 
   CollisionManager::registerType<Drone>({typeid(Tree).name()});
   CollisionManager::registerType<Tree>({typeid(Drone).name()});

   // Register our custom type with the TargetFactory. We supply it with template parameters corresponding to the **unique** parameters in our target class (AKA not the three that need to be there)
   // It takes function parameters equal to the name of the type, through swarm::get_type<T>
   TargetFactory::registerTargetType<MyTarget, bool, bool, float>(get_type<MyTarget>);
   ```
Now the type is fully registered and can be included in the simulation through TestConfigs like as follows:
   ```cpp
     swarm::TestConfig config = {"My Alg",
                              behaviour_parameters,
                              drone_config,
                              map,
                              drone_count,
                              target_count,
                              time_limit,
                              get_type<MyTarget>(), // The target type name requested
                              std::tuple(true, 5.0f, 3),   // an std::tuple of the parameters for the target
                              contact_listener.get()};
   ```

</details>

<details>

<summary>Adding custom Contact Listeners</summary>

### Adding a new Contact Listener
After registering types as shown above, collisions between them can be controlled via the `swarm::BaseContactListener` class. (This class could also be extended, if the user wishes)
To do this, in the main testbed entrypoint `main.cpp`, create a new shared instance of `swarm::BaseContactListener`:
   ```cpp
     auto contact_listener = std::make_shared<BaseContactListener>();
   ```
Then, add collision handlers to it using the `addCollisionHandler` function. This function takes two types, which can be retrieved via `swarm::get_type<Class Name>`, and a user defined function. This function needs to be `void`, and takes two function parameters: `b2Fixture*` and `b2Fixture*`. These two fixtures represent the two fixtures that have been found to be colliding.

Here's an example of adding a collision handler between a `Drone` and a `MyTarget`, using a lambda function:
   ```cpp

   void collideDroneMyTarget(b2Fixture* drone_fixture, b2Fixture* target_fixture) {
        // Check if the drone fixture is a sensor, we don't want to do this if it's the physical drone
     if (drone_fixture.isSensor()) {
         Drone *drone = reinterpret_cast<UserData *>(
                                  drone_fixture->GetUserData().pointer)
                                  ->as<Drone>();
         MyTarget *my_target = reinterpret_cast<UserData *>(
                         target_fixture->GetUserData().pointer)
                         ->as<MyTarget>();
         my_target->DoSomething();
         drone->addTargetFound(my_target);
         std::cout << drone->getId() << " found " << std::cout << my_target->getId() << std::endl;
      }
   }
   contact_listener->addCollisionHandler(
         get_type<Drone>, get_type<MyTarget>,
         collideDroneMyTarget
         );
   ```
   - This can also be done inline using lambda functions:
   ```cpp
      contact_listener->addCollisionHandler(
         get_type<Drone>, get_type<MyTarget>,
         [](b2Fixture* a, b2Fixture* b) -> void {
            // Do something with a and b
         });
   ```

We then pass this listener into the TestConfig for it to be used by a simulation.

</details>

<details>

<summary>Adding custom Drone Configurations</summary>
  
### Adding a new Drone Configuration
Drone configurations are simple to add. We simply register them with the configuration register.

</details>

 
## Example Testbed Configuration

```user.cpp
// user.cpp
#include "testbed.h"

// Create Parameters for CustomBehaviour
auto behaviour_parameters = std::make_unique<CustomBehaviour>(250.0, 1.6, 1.0, 3.0, 3.0);

// Create a drone configuration
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
auto target_parameters = std::make_tuple(...);

swarm::TestConfig test = {
   "Custom Behaviour",
   behaviour_parameters,
   drone_config,
   map,
   num_drones,
   num_targets,
   target_parameters,
   listener.get(),
   time_limit,
};

// Create queue and add test to it
swarm::TestQueue queue;
queue.push(test);

// Begin the simulation 
testbed::run();
```

## Architecture
TODO: is this necessary? If so, I will complete it
