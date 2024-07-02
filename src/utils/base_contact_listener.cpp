#include "salsa/utils/base_contact_listener.h"

#include <string>

#include "box2d/b2_body.h"
#include "salsa/core/logger.h"
#include "salsa/entity/entity.h"
#include "salsa/utils/object_types.h"
#include "spdlog/spdlog.h"
namespace swarm {
std::vector<BaseContactListener *> BaseContactListener::registry_;

void BaseContactListener::addCollisionHandler(
    std::string type1, std::string type2,
    std::function<void(b2Fixture *, b2Fixture *)> handler) {
  get_logger()->info("Adding collision handler for types: {} and {}", type1,
                     type2);
  if (handler) {
    get_logger()->info("Handler is not null");
  }
  if (collision_handlers_.size() == 0) {
    get_logger()->info("Collision handlers map is empty");
  }
  collision_handlers_[{type1, type2}] = handler;

  for (auto &handler : collision_handlers_) {
    get_logger()->info("Handler: {} and {}", handler.first.first,
                       handler.first.second);
  }

  // Ensure symmetric handling, no matter the order of contact
  // Unless they're already the same type, as we can handle both then
  if (type1 != type2) {
    collision_handlers_[{type2, type1}] =
        [handler](b2Fixture *a, b2Fixture *b) { handler(a, b); };
  }
}

void BaseContactListener::BeginContact(b2Contact *contact) {
  b2Fixture *fixtureA = contact->GetFixtureA();
  b2Fixture *fixtureB = contact->GetFixtureB();

  // Only consider interactions between two dynamic bodies
  UserData *userDataA =
      reinterpret_cast<UserData *>(fixtureA->GetUserData().pointer);
  UserData *userDataB =
      reinterpret_cast<UserData *>(fixtureB->GetUserData().pointer);

  if (!userDataA || !userDataB) {
    return;
  }
  if (!userDataA->object || !userDataB->object) {
    get_logger()->error("One of the UserData objects is null.");
    return;
  }
  auto handlerIt =
      collision_handlers_.find({demangle(typeid(*(userDataA->object)).name()),
                                demangle(typeid(*(userDataB->object)).name())});

  if (handlerIt != collision_handlers_.end()) {
    handlerIt->second(fixtureA, fixtureB);
  } else {
    get_logger()->error("No collision handler for types {} and {}",
                        demangle(typeid(*(userDataA->object)).name()),
                        demangle(typeid(*(userDataB->object)).name()));
  }
}
}  // namespace swarm