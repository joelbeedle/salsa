#include "utils/base_contact_listener.h"

#include "box2d/b2_body.h"
#include "spdlog/spdlog.h"
#include "utils/object_types.h"
namespace swarm {
void BaseContactListener::addCollisionHandler(
    std::string type1, std::string type2,
    std::function<void(b2Fixture *, b2Fixture *)> handler) {
  std::cout << "Adding collision handler for types: " << type1 << " and "
            << type2 << std::endl;
  if (handler) {
    std::cout << "Handler is not null" << std::endl;
  }
  if (collision_handlers_.size() == 0) {
    std::cout << "Collision handlers is empty" << std::endl;
  }
  collision_handlers_[{type1, type2}] = handler;
  std::cout << "Collision handlers size: " << collision_handlers_.size()
            << std::endl;
  for (auto &handler : collision_handlers_) {
    std::cout << "Handler: " << handler.first.first << " and "
              << handler.first.second << std::endl;
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
    std::cout << "Error: One of the UserData objects is null." << std::endl;
    return;
  }
  auto handlerIt =
      collision_handlers_.find({demangle(typeid(*(userDataA->object)).name()),
                                demangle(typeid(*(userDataB->object)).name())});

  if (handlerIt != collision_handlers_.end()) {
    handlerIt->second(fixtureA, fixtureB);
  } else {
    std::cout << "No collision handler for types "
              << demangle(typeid(*(userDataA->object)).name()) << " and "
              << demangle(typeid(*(userDataB->object)).name()) << std::endl;
  }
}
}  // namespace swarm