#ifndef SWARM_SIM_CORE_SIMULATION_H
#define SWARM_SIM_CORE_SIMULATION_H
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include "box2d/box2d.h"
#include "salsa/behaviours/behaviour.h"
#include "salsa/behaviours/parameter.h"
#include "salsa/behaviours/registry.h"
#include "salsa/core/data.h"
#include "salsa/core/logger.h"
#include "salsa/core/map.h"
#include "salsa/core/sim.h"
#include "salsa/core/test_queue.h"
#include "salsa/entity/drone.h"
#include "salsa/entity/drone_configuration.h"
#include "salsa/entity/drone_factory.h"
#include "salsa/entity/entity.h"
#include "salsa/entity/target.h"
#include "salsa/utils/base_contact_listener.h"
#include "salsa/utils/collision_manager.h"
#include "salsa/utils/object_types.h"
#include "salsa/utils/raycastcallback.h"
#endif  // SWARM_SIM_CORE_SIMULATION_H