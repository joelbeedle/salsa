[![DOI](https://zenodo.org/badge/721758864.svg)](https://zenodo.org/doi/10.5281/zenodo.13151118)

# SALSA - Swarm Algorithm Simulator

**S**warm **Al**gorithm **S**imul**a**tior Library and Testbed, written in C++.

**SALSA** is intended for use primarily in swarm behavior algorithm testing. Ideally, it fits early-on in the process of developing software for swarms of drones, helping users to develop algorithms, and view prelimiary tests on algorithm effectiveness and correctness. After verification, depending on user requirements, could either stick with **SALSA**, maybe even extending it for their specific needs (such as adding their drone specifications, adding a movement model, or even a new testbed using the Library), or they could move on to more sophisticated software, now that they know what their algorithm **should** look like when performing correctly.

[SALSA Testbed Demonstration video](https://youtu.be/K64oUI9zK0c)

## Getting Started

There is a Docker container available. It opens a noVNC web app on `localhost` with `salsa` pre-installed.

### Using Docker
#### Prebuilt Docker container

This docker container **only** contains what is in the repository. If you want to extend the program, you need to clone the repository and build the docker container / the code.

- **Run the docker image:**

```bash
docker run --shm-size=256m -it -p 5901:5901 -e VNC_PASSWD=123456 ghcr.io/joelbeedle/salsa:latest
```

#### Building the Docker container yourself

- **Clone the repository:**

```bash
git clone --recursive https://github.com/joelbeedle/salsa.git
git submodule update --init --recursive
```

- **Build the docker image:**

```bash
docker build -t salsa-test/testbed .
```

- **Run the container:**

```bash
docker run --shm-size=256m -it -p 5901:5901 -e VNC_PASSWD=123456 salsa-test/testbed
```

Using either method, to use the virtual machine:

- Go to `http://localhost:5901` and enter the password `123456`

- Right click, select `Applications > Shells > Bash` and you will find yourself already in a terminal in the repository.

- The testbed application can be found at `./build/testbed/`

- More information on how to use the testbed can be found [here](#using-the-testbed)

### Building it yourself

`salsa` was designed with to be cross-platform, for Windows, Linux, and macOS.

#### Requirements

- CMake **3.14+**
- A **C++17** compatible compiler (gcc, clang, cl)
- Git
- OpenGL
- Python3
- Optional: [Doxygen](https://github.com/doxygen/doxygen), [Ninja](https://github.com/ninja-build/ninja)

Requirements in bold are **essential**.

`salsa` uses the following packages directly:

- [Box2D](https://github.com/erincatto/box2d)
- [GLAD](https://github.com/Dav1dde/glad) and [GLFW](https://www.glfw.org/)
- [spdlog](https://github.com/gabime/spdlog)
- [nlohmann_json](https://github.com/nlohmann/json)
- [CLI11](https://github.com/CLIUtils/CLI11)
- [ImGui](https://github.com/ocornut/imgui)
- [googletest](https://github.com/google/googletest)

These requirements are all either managed as git submodules, or are fetched and installed into `build/_deps` when configuring using CMake's [FetchContent](https://cmake.org/cmake/help/latest/module/FetchContent.html) module automatically, so you shouldn't have to do anything to set them up.

#### Steps

- **Clone the repository and all submodules:**
  ```bash
  git clone --recursive https://github.com/joelbeedle/salsa.git
  ```
  > **Note**: the `--recursive` tag is important, as this also clones the submodules, which are needed.
- **Initialise and update the submodules:**
  ```bash
  git submodule update --init --recursive
  ```
- **Configure the project with CMake:**

  ```bash
  cd salsa
  cmake -S . -B build
  ```

  > If you have Ninja installed, for a faster build time use:
  >
  > ```bash
  >  cmake -S . -B build -GNinja
  > ```

- **Build the project:**
  ```bash
  cmake --build build
  ```
- **To build documentation (requires Doxygen):**
  ```bash
  cmake --build build --target docs
  ```
  > Documentation can then be found in `build/docs/html/index.html`

> There are also some common build configurations found in `CMakePresents.json`.

### Running Testbed

To run the testbed:

- Navigate to the `build/testbed` directory and execute `./testbed`.
- Use `./testbed --help` for a list of command line parameters

The testbed, when opened, presents the main menu. From here, we can access the following features:

- **Simulators**
  - [Queue Mode](#queue-mode)
  - [Sandbox Mode](#sandbox-mode)
- [**Map Creator**](#map-creator)

## Using the Testbed

The testbed is extended from the original Box2D testbed. It has three interactive GUI modes, and a headless mode. The three GUI modes are Sandbox, Queue, and Map Creator.

The testbed comes with a template `user.cpp` file, containing an example on how to configure the testbed and use the Test Queue. There is more detail on how to create custom implementations in the [Extensibility section](#extensibility).

Both Sandbox and Queue modes allow the user to dynamically interact with the simulation as it is occuring. Queue mode is intended to collect data, and Sandbox mode is intended for algorithm development. Modes can be switched between seamlessly. Registries are used internally inside the `salsa` library as communication interfaces.

Headless mode can be entered using `./testbed --headless`. Headless mode runs much faster than the visual modes, as it is intended to iterate through simulations, outputting data. Verbose mode can be selected using `-v`, and a `.json` queue file (generated from the testbed) to use can be specified using `-q`. The `.json` file extension does not need to be included, but if it is, the program still runs.

> Example: `./testbed --headless -v -q example_queue` 

An example of how to use **Sandbox** and **Queue** is shown in the [demonstration video](https://www.youtube.com/watch?v=K64oUI9zK0c)

### Queue Mode
<img src="./docs/imgs/boxplot.jpg" width="500">

We used the Queue mode to generate the data used for the plot above. To run a similar queue (bar the Levy Flocking), you can load [example queue](./example_queue.json).

**Features:**
- Create a test queue of various simulations and parameters, and get output data. Data is logged to the `testbed/results` folder.
- After the time limit for each runs out, plots are created from the data automatically.
  > In headless mode, all plots are generated unless specified otherwise.
- The queue can be set beforehand in `user.cpp`, or generated in the GUI.
  - To generate in the GUI: go to the Test Queue section, click the `+` button, and choose whether to add a single test or add a group of tests, permuting behavior parameters.

#### Saving and Loading Queues

- Queues can be saved and loaded through the GUI, and also in user code, using `TestQueue::load(filename)` and `TestQueue::save(filename)`. A saved queue can also be independently loaded in headless mode using the `-q` option.

#### Performance Analysis

- The real-time factor performance of the simulator is assessed when ran headless and using the `-v` verbose setting. The simulator will then go through the test queue, and for each entry, when the test is complete, calculates the RTF. It outputs this into `results/` with the `.csv` filename the same as the test queue configuration file.

- To generate the data shown in `Table 1` of the paper:
  - **Ensure that the** `.csv` **file is empty, or deleted**
  - **Build the simulator**
  - **Run the simulator headless, with verbosity, no plots, and specify the automatically generated test queue:** `./testbed --headless -v --no-plots -q performance_table_queue`
  - **wait until this is complete**
  - **Go to** `testbed/plot/`
  - **Run** `python3 table.py performance_table_queue.csv`
  - **The table is displayed in the console**

### Sandbox Mode

Sandbox mode is used to test a behavior, without having to create tests. It is intended as a way for users to visually interact with their algorithms, and assess if they are functioning correctly.

**Features:**
- Dynamically change parameters, maps, drone configurations, etc., on the go.
- Change swarm sizes, map, behaviour, and any parameters, and get instant feedback on what the changes do.

### Map Creator

The Map Creator allows the user to create their own maps. The menu on the left is the main interface.

- **Drawing Tips:**
  - Drawing lines is done by clicking and holding, then dragging and releasing to end the line
  - Drawing shapes (hollow polygon, polygon) is done by clicking once, releasing, then clicking again to set the next vertex. Connect the last vertex to the first to draw the shape.
  - The drone spawn point should be set for where you want drones to spawn.

    > Drones wont spawn inside solid objects, but they will spawn inside hollow polygons.

  - A square world boundary can be set by checking Draw Boundary and adjusting the size of the sides.
  - Press `Q` to `undo` the last drawing action, or reset the cursor.

#### Saving and Loading Maps
- **To save**, click `File > Save As` and then enter a name when prompted. This will refresh the map registry, so if you want to use the map in a simulation straight away, you can.
- **To load**, click `File > Load`, and select a map to load.

## Extensibility

SALSA is designed with extensibility at it's heart - it works best when it's utilised and moulded to whatever the user wants.

The testbed can be extended in multiple ways: users can create custom swarm algorithms, targets, contact listeners and managers, and drone configurations, alongside custom maps and data. New testbed modes could be added, similar to the current `Queue` and `Sandbox` modes. Furthermore, as the Library function is decoupled from our Testbed, a new testbed could be created entirely: perhaps targeting ground based robots, utilising the framework the Library provides in the same ways that our Testbed does.

> [!IMPORTANT]
> **Any new files need to be added to the testbed `CMakeLists.txt` file, in `MY_TESTBED_SOURCE_FILES`.**

For more information and examples on extending the various aspects of the testbed, expand a section below.

<details>

<summary>Adding new swarm Algorithms</summary>

### Add a Custom swarm Algorithm

To add a custom swarm algorithm:
Create a new `.cpp` file inside `testbed/behaviors`, and name it, etc `my_alg.cpp`.

Inside this file, import `<salsa/salsa.h>` to import all library headers. (The following examples are as if the code is written inside the `salsa` namespace, for ease of reading)

Create a class that extends the `Behaviour` class:

```cpp
#include <salsa/salsa.h>

class MyAlg : public Behaviour {};
```

Define any parameters as a behaviour::Parameter

```cpp
class MyAlg : public Behaviour {
  private:
   behaviour::Parameter param_1_;
   behaviour::Parameter param_2_;
   behaviour::Parameter param_3_;
};
```

Implement a constructor that takes as arguments the behaviour parameters, and pass these into the defined `behaviour::Parameter`s in the constructor initializer list.

Then, create a key value pair in `parameters_`, a map of names to parameters, using as a key the display name, and as value a reference to the internal `behaviour::Parameter`.

```cpp
 class MyAlg : public Behaviour {
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
auto alg = behaviour::Registry::get().add(
    "My Alg",
    std::make_unique<MyAlg>(param_1, param_2));
```

This behaviour will now appear in the testbed, and can be set in a TestConfig using the behaviour namme specified, e.g. `My Alg`

</details>

<details>
  
<summary>Adding new Targets</summary>

### Add a custom Target

Create a `.cpp` and `.h` file in `testbed/targets`, and name it, etc. `my_target.h`.

In the header file, include `<core/simulation.h>` and create a class that extends `salsa::Target`

```cpp
class MyTarget : public Target {};
```

In its constructor, a target **must** have the first 3 parameters as `b2World* world, const b2Vec2& position, int id`, but any other parameters are user definable.

```cpp
MyTarget(b2World* world, const b2Vec2 &position, int id, bool visible, float radius, int arg) {}
```

A target should follow this pattern for its body definitions:

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

  // We then create a salsa::UserData object. This is metadata and how collision callbacks are managed.
  UserData *my_data = new UserData();
  // Set the userData's object argument to point to this instance
  my_data->object = this;
  // Set the userData of the fixtureDef as a pointer to our salsa::UserData object and create fixture
  fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(my_data);
  body_->CreateFixture(&fixtureDef);
```

We don't need to worry about creating the body, as this is done through the `Entity` class that `Target` inherits from. This is done to simplify creation and abstract the physics engine interface from the user.
Once the target is fully defined, we register it with the `TargetFactory` and the collision manager, in our `user.cpp`.

```cpp
// user.cpp
// If we want drones to detect this target type and the target type to also detect the trees, define both.
// If not, `registerType` should take < The thing you want to do the detecting >
CollisionManager::registerType<Drone>({typeid(Tree).name()});
CollisionManager::registerType<Tree>({typeid(Drone).name()});

// Register our custom type with the TargetFactory. We supply it with template parameters corresponding to the **unique** parameters in our target class (AKA not the three that need to be there)
TargetFactory::registerTarget<MyTarget, bool, bool, float>("Target Instance Name", false, false, true);
```

Now the type is fully registered and can be included in the simulation through TestConfigs like as follows:

```cpp
  salsa::TestConfig config = {"My Alg",
                           behaviour_parameters,
                           drone_config,
                           map_name,
                           drone_count,
                           target_count,
                           time_limit,
                           "Target Instance Name"
                           contact_listener_name};
```

</details>

<details>

<summary>Adding custom Contact Listeners</summary>

### Adding a new Contact Listener

After registering types as shown above, collisions between them can be controlled via the `salsa::BaseContactListener` class. (This class could also be extended, if the user wishes)
To do this, in the main testbed entrypoint `main.cpp`, create a new static shared pointer of `salsa::BaseContactListener` (it has to be static so it can be accessed from other scopes), and give it a name:

```cpp
  static auto contact_listener = std::make_shared<BaseContactListener>("Name");
```

Then, add collision handlers to it using the `addCollisionHandler` function. This function takes two types, which can be retrieved via `salsa::get_type<Class Name>`, and a user defined function. This function needs to be `void`, and takes two function parameters: `b2Fixture*` and `b2Fixture*`. These two fixtures represent the two fixtures that have been found to be colliding.

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

We then pass the listener's name into the `TestConfig` for it to be used by a simulation.

</details>

<details>

<summary>Adding custom Drone Configurations</summary>
  
### Adding a new Drone Configuration
Drone configurations are simple to add. We simpy create one in `user.cpp`, give it a name, and it will appear in the testbed.

```cpp
salsa::DroneConfiguration *drone_config = new salsa::DroneConfiguration(
    "Config Name", 15.0f, 50.2f, 10.0f, 0.3f, 1.0f, 1.5f, 134.0f);
```

</details>

## Example Testbed Configuration

```cpp
// user.cpp
#include "testbed.h"

using namespace salsa;

void user() {
  // Get the default parameters for CustomBehaviour
  auto behaviour_parameters = behaviour::Registry::get().behaviour("Custom Behaviour")->getParameters();

  // Create a drone configuration
  auto *drone_config = new DroneConfiguration("My Config",
      25.0f, 50.0f, 10.0f, 0.3f, 1.0f, 1.5f, 4000.0f);

  // Set up the listener to listen for collisions.
  // Has to be static, and named
  static auto listener = std::make_shared<BaseContactListener>("Default");

  // Register the drones and targets with the collision manager
  CollisionManager::registerType<Drone>({typeid(CustomTarget).name()});
  CollisionManager::registerType<CustomTarget>({typeid(Drone).name()});

  // Add the target to the TargetFactory, supplying any custom parameters and their types
  // This can be done multiple times for the same Target class, with a different name.
   TargetFactory::registerTarget<CustomTarget, bool, bool, float>("Custom_1", false,
                                                                false, 5.0f);

  // Add a collision handler between Drone and Target types, when this collision is detected, userHandlingFunction is called to handle the collision.
  listener.addColisionHandler(typeid(Drone), typeid(CustomTarget), userHandlingFunction)

  // Set up test environment
  auto map_name = "test" // "test.json" here is a map that has already been created and is in testbed/maps
  auto num_drones = 100;
  auto num_targets = 1000;
  auto time_limit = 1200.0;

  TestConfig test = {
     "Custom Behaviour",
     behaviour_parameters,
     drone_config,
     map_name,
     num_drones,
     num_targets,
     target_type,   // "Custom_1"
     listener_name, // "Default"
     time_limit,    // 100.0
  };

  // Add test to the Simulator's TestQueue
  TestQueue::push(test);

  // Or... load a queue that you've saved before:
  TestQueue::load("my_queue");
}
```

This code will not compile. For an example that sets up a test queue, check the real `testbed/user.cpp` file.

## Contributions
Contributions are welcome for SALSA! Please see the [Contributing to SALSA guide](https://github.com/joelbeedle/salsa/blob/master/.github/CONTRIBUTING.md) for more information.

## License
SALSA is licensed under the [zlib License](https://www.zlib.net/zlib_license.html)
