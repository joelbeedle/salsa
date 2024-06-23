#include "utils/base_contact_listener.h"

#include "box2d/b2_body.h"
#include "spdlog/spdlog.h"
#include "utils/object_types.h"
namespace swarm {
void BaseContactListener::addCollisionHandler(
    std::type_index type1, std::type_index type2,
    std::function<void(b2Fixture *, b2Fixture *)> handler) {
  spdlog::info("Adding collision handler for types: {} and {}", type1.name(),
               type2.name());
  if (handler) {
    spdlog::debug("Handler is not null");
  }
  if (collision_handlers_.size() == 0) {
    spdlog::debug("Collision handlers is empty");
  }
  collision_handlers_[{type1, type2}] = handler;
  spdlog::info("Collision handlers size: {}", collision_handlers_.size());

  // Ensure symmetric handling, no matter the order of contact
  // Unless they're already the same type, as we can handle both then
  if (type1 != type2) {
    collision_handlers_[{type2, type1}] =
        [handler](b2Fixture *a, b2Fixture *b) { handler(b, a); };
  }
}

void BaseContactListener::BeginContact(b2Contact *contact) {
  b2Fixture *fixtureA = contact->GetFixtureA();
  b2Fixture *fixtureB = contact->GetFixtureB();

  // Only consider interactions between two dynamic bodies
  if (fixtureA->GetBody()->GetType() == b2_staticBody ||
      fixtureB->GetBody()->GetType() == b2_staticBody) {
    return;
  }
  UserData *userDataA =
      reinterpret_cast<UserData *>(fixtureA->GetUserData().pointer);
  UserData *userDataB =
      reinterpret_cast<UserData *>(fixtureB->GetUserData().pointer);

  if (!userDataA || !userDataB) {
    spdlog::error("Error: Missing UserData in one of the fixtures.");
    return;
  }
  if (!userDataA->object || !userDataB->object) {
    spdlog::error("Error: One of the UserData objects is null.");
    return;
  }
  auto handlerIt = collision_handlers_.find(
      {typeid(*(userDataA->object)), typeid(*(userDataB->object))});

  if (handlerIt != collision_handlers_.end()) {
    handlerIt->second(fixtureA, fixtureB);
  } else {
    spdlog::warn("No collision handler for types {} and {}",
                 type(*userDataA->object), type(*userDataB->object));
  }
}
}  // namespace swarm