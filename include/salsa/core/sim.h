/// @file sim.h
/// @brief This file contains the declaration of the Sim class, which is the
/// main class for the simulation. It defines a `Simulation`, which is the
/// foundation of the SALSA library.
#ifndef SWARM_SIM_CORE_SIM_H
#define SWARM_SIM_CORE_SIM_H

#include <box2d/box2d.h>

#include <sstream>
#include <variant>

#include "salsa/behaviours/behaviour.h"
#include "salsa/behaviours/registry.h"
#include "salsa/entity/drone.h"
#include "salsa/entity/drone_configuration.h"
#include "salsa/entity/drone_factory.h"
#include "salsa/entity/target.h"
#include "salsa/entity/target_factory.h"
#include "salsa/utils/base_contact_listener.h"
#include "test_queue.h"
namespace salsa {

/// @brief Defines the parameters for a drone in the simulation.
struct DroneParameters {
  float cameraViewRange;
  float obstacleViewRange;
  float droneDetectionRange;
  float maxSpeed;
  float maxForce;
  float mass;
  float radius;
};

/// @brief The main class for the simulation.
class Sim {
 private:
  map::Map map_;    ///< The map of the simulation environment
  b2World* world_;  ///< The Box2D world for the simulation
  salsa::BaseContactListener*
      contact_listener_;  ///< The contact listener for the simulation

  /// @name Simulation properties
  /// These properties originate from the test configuration and are used to
  /// set up the simulation.
  ///@{
  salsa::TestConfig
      test_config_;  ///< The test configuration for the simulation
  float border_height_;
  float border_width_;
  b2Vec2 drone_spawn_position_;
  float time_limit_ = -1.0f;
  float current_time_ = 0.0f;
  bool is_stack_test_ = false;
  int num_time_steps_ = 0;
  ///@}

  /// @name Drone properties
  /// These properties are used to manage the drones in the simulation.
  ///@{
  /// Map of all drone parameters by name.
  std::unordered_map<std::string, DroneParameters> all_drone_parameters_;
  /// Map of all drone configurations by name.
  std::unordered_map<std::string, salsa::DroneConfiguration>
      all_drone_configurations_;
  /// The current drone configuration.
  salsa::DroneConfiguration* drone_configuration_;
  /// The list of drones in the simulation.
  std::vector<std::unique_ptr<salsa::Drone>> drones_;
  int num_drones_;   ///< The number of drones in the simulation
  float max_speed_;  ///< The maximum speed of the drones
  float max_force_;  ///< The maximum force of the drones
  ///@}

  /// @name Target properties
  ///@{
  /// @brief The type of target in the simulation.
  /// This is used to determine the type of target to create.
  std::string target_type_;

  /// The list of targets in the simulation.
  std::vector<std::shared_ptr<Target>> targets_;
  /// The list of targets found in the current time step.
  std::vector<Target*> targets_found_this_step_;
  float num_targets_;  ///< The number of targets in the simulation
  ///@}

  // Obstacles in the environment
  std::vector<b2Body*> obstacles_;
  float obstacle_view_range_;

  // Behaviour management
  salsa::Behaviour* behaviour_;
  std::string current_behaviour_name_;
  float camera_view_range_;

  // Visualization flags
  bool draw_visual_range_ = false;
  bool draw_targets_ = false;
  bool draw_drones_ = false;

  // Logging
  Logger& logger_ = Logger::getInstance();
  std::chrono::steady_clock::time_point last_log_time;
  std::vector<std::shared_ptr<Observer>> observers_;
  int log_interval_ = 5;  // time steps
  // Private methods for internal use
  void createBounds();
  void applyCurrentBehaviour();
  void createDronesCircular(Behaviour& behaviour,
                            DroneConfiguration& configuration);
  void createDronesRandom(Behaviour& behaviour,
                          DroneConfiguration& configuration);

 public:
  // Constructors and Destructor
  Sim(b2World* world, int drone_count, int target_count,
      DroneConfiguration* config, float border_width, float border_height,
      float time_limit);
  Sim(salsa::TestConfig& config);
  ~Sim();

  enum class SpawnType { CIRCULAR, RANDOM };

  /// @name Simulation Control
  /// These functions control the simulation.
  ///@{
  /// @brief Runs the simulation for a single time step.
  void update();

  /// @brief Resets the simulation to its initial state.
  void reset();

  /// @brief Adds an observer to the simulation.
  /// @param observer The observer to add.
  void addObserver(std::shared_ptr<Observer> observer) {
    observers_.push_back(observer);
  }
  ///@}

  /// @name Behaviour Functions
  /// Functions dedicated to Behaviour management
  ///@{
  /// @brief Adds a behaviour to the simulation.
  void addBehaviour(const std::string& name,
                    std::unique_ptr<salsa::Behaviour> behaviour);

  /// @brief Sets the current behaviour of the simulation.
  /// @param behaviour A pointer to the behaviour to set.
  void setCurrentBehaviour(Behaviour* behaviour);

  /// @brief Sets the current behaviour of the simulation by name.]
  /// The function will search the Behaviour Registry for a behaviour matching
  /// that name, and then set the current behaviour to that.
  /// @param name The name of the behaviour to set.
  void setCurrentBehaviour(const std::string& name);
  ///@}

  /// @name Drone Functions
  /// Functions dedicated to Drone management
  ///@{

  /// @brief Creates drones in the simulation.
  /// @param behaviour The behaviour to initialize the drones with.
  /// @param configuration The drone configuration to initialize the drones
  /// with.
  /// @param mode The mode to spawn the drones in. Can either be `CIRCULAR` or
  /// `RANDOM`.
  void createDrones(Behaviour& behaviour, DroneConfiguration& configuration,
                    SpawnType mode);

  void setDroneCount(int count);
  int getDroneCount();
  int& num_drones();
  const int& num_drones() const;
  void setDroneConfiguration(DroneConfiguration* configuration);

  /// @brief Updates the drone settings in the simulation.
  /// Iterates through each drone in the simulation and updates their max force,
  /// max speed, view range, obstacle range, drone detection range, and sensor
  /// range, to be consistent with the current simulation drone configuration.
  void updateDroneSettings();
  std::vector<std::unique_ptr<salsa::Drone>>& getDrones();

  /// @brief Sets the vector of drones to the simulation.
  /// Care must be taken to ensure that the drones inhabit the same `b2World` as
  /// the simulation.
  /// @param drones The vector of drones to set.
  void setDrones(std::vector<std::unique_ptr<salsa::Drone>> drones);
  ///@}

  /// @name Target Functions
  ///@{

  /// @brief Creates multiple target objects within the simulation.
  ///
  /// This method generates a specified number of targets at random positions
  /// within the given borders. Each target is initialized through the
  /// TargetFactory and added to the simulation's target list.
  ///
  /// @tparam Params Variadic template parameters to pass to the target factory.
  /// @param params Parameters required for creating a target, passed variably.
  template <typename... Params>
  void createTargets(Params... params);

  /// @brief Sets the type of targets to be created in the simulation.
  ///
  /// This method updates the target type, influencing how future targets
  /// are created and their behaviors. It searches for a registered target with
  /// the name `type_name`.
  ///
  /// @param type The type of target as a string identifier
  void setTargetType(const std::string& type_name);

  /// @brief Sets the number of targets to be created in the simulation.
  ///
  /// Specifies the total number of target objects that should be initialized
  /// in the next creation cycle.
  ///
  /// @param count The number of targets to create.
  void setTargetCount(int count);

  /// @brief Get a vector of all targets in the simulation.
  /// @return A vector of shared pointers to all targets in the simulation.
  std::vector<std::shared_ptr<Target>>& getTargets();

  /// @brief Get a vector of all targets found precisely in this simulation
  /// step.
  /// @return A vector of normal pointers to all targets in the simulation
  std::vector<Target*>& getTargetsFoundThisStep();

  /// @brief Multi-threaded function to get the number of targets found.
  /// @return The number of targets in the simulation with `isFound()` as true.
  int countFoundTargets();
  ///@}

  /// @brief Sets the contact listener for the simulation.
  /// @param listener The contact listener to set.
  void setContactListener(BaseContactListener& listener);

  // Getters and Setters for properties
  float& world_height();
  const float& world_height() const;
  float& world_width();
  const float& world_width() const;
  float& current_time();
  const float& current_time() const;
  float& time_limit();
  b2World* getWorld();
  void setWorld(b2World* world);
  map::Map getMap();
  salsa::TestConfig& test_config();
  std::string& current_behaviour_name();
  const std::string& current_behaviour_name() const;
  DroneConfiguration* getDroneConfiguration();
  const DroneConfiguration* drone_configuration() const;
  void setCurrentDroneConfiguration(DroneConfiguration& configuration);
  void changeMap(std::string name);
};

/// @brief Builder class for the Sim class.
/// This class provides a fluent interface for building a Sim object. It is not
/// used currently, but is provided for future extensibility.
class SimBuilder {
 private:
  b2World* world_;
  Behaviour* behaviour_;
  int drone_count_ = 0;
  int target_count_ = 0;
  float world_height_ = 0;
  float world_width_ = 0;
  float time_limit_ = -1.0f;
  DroneConfiguration* config_;
  BaseContactListener* contact_listener_;

 public:
  SimBuilder& setWorld(b2World* world) {
    world_ = world;
    return *this;
  }

  SimBuilder& setDroneCount(int count) {
    drone_count_ = count;
    return *this;
  }

  SimBuilder& setTargetCount(int count) {
    target_count_ = count;
    return *this;
  }

  SimBuilder& setContactListener(BaseContactListener& listener) {
    contact_listener_ = &listener;
    world_->SetContactListener(&listener);
    return *this;
  }

  SimBuilder& setDroneConfiguration(DroneConfiguration* config) {
    config_ = config;
    return *this;
  }

  SimBuilder& setWorldHeight(float height) {
    world_height_ = height;
    return *this;
  }

  SimBuilder& setWorldWidth(float width) {
    world_width_ = width;
    return *this;
  }

  SimBuilder& setTimeLimit(float time_limit) {
    time_limit_ = time_limit;
    return *this;
  }

  Sim* build() {
    std::cout << "Building sim with:\nDrone count: " << drone_count_
              << "\nTarget Count: " << target_count_
              << "\nWorld Height: " << world_height_
              << "\nWorld Width: " << world_width_
              << "\nConfig: " << (config_ != nullptr) << std::endl;
    return new Sim(world_, drone_count_, target_count_, config_, world_width_,
                   world_height_, time_limit_);
  }
};

}  // namespace salsa

#endif  // SWARM_SIM_CORE_SIM_H
