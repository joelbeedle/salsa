#include "utils/base_contact_listener.h"

#include "utils/object_types.h"

namespace swarm {
void BaseContactListener::addCollisionHandler(
    std::type_index type1, std::type_index type2,
    std::function<void(b2Fixture *, b2Fixture *)> handler) {
  std::cout << "Adding collision handler for types: " << type1.name() << " and "
            << type2.name() << std::endl;
  if (handler) {
    std::cout << "Handler is not null" << std::endl;
  }
  if (collision_handlers_.size() == 0) {
    std::cout << "Collision handlers is not null" << std::endl;
  }
  collision_handlers_[{type1, type2}] = handler;
  std::cout << "We added it!" << std::endl;

  // Ensure symmetric handling, no matter the order of contact
  collision_handlers_[{type2, type1}] = [handler](b2Fixture *a, b2Fixture *b) {
    handler(b, a);
  };
}

void BaseContactListener::BeginContact(b2Contact *contact) {
  b2Fixture *fixtureA = contact->GetFixtureA();
  b2Fixture *fixtureB = contact->GetFixtureB();

  UserData *userDataA =
      reinterpret_cast<UserData *>(fixtureA->GetUserData().pointer);
  UserData *userDataB =
      reinterpret_cast<UserData *>(fixtureB->GetUserData().pointer);

  if (!userDataA || !userDataB) {
    std::cerr << "Error: Missing UserData in one of the fixtures." << std::endl;
    return;
  }
  if (!userDataA->object || !userDataB->object) {
    std::cerr << "Error: One of the UserData objects is null." << std::endl;
    return;
  }
  auto handlerIt = collision_handlers_.find(
      {typeid(*(userDataA->object)), typeid(*(userDataB->object))});

  if (handlerIt != collision_handlers_.end()) {
    handlerIt->second(fixtureA, fixtureB);
  } else {
    std::cerr << "No collision handler for the given types." << std::endl;
  }
}
}  // namespace swarm