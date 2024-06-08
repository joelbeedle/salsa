#ifndef SWARM_UTILS_BASE_CONTACT_LISTENER_H
#define SWARM_UTILS_BASE_CONTACT_LISTENER_H

#include <box2d/box2d.h>

#include <functional>
#include <map>
#include <typeindex>

namespace swarm {

class BaseContactListener : public b2ContactListener {
 protected:
  std::map<std::pair<std::type_index, std::type_index>,
           std::function<void(b2Fixture*, b2Fixture*)>>
      collision_handlers_;

 public:
  void addCollisionHandler(std::type_index type_a, std::type_index type_b,
                           std::function<void(b2Fixture*, b2Fixture*)> handler);
  void BeginContact(b2Contact* contact) override;
};

}  // namespace swarm

#endif  // SWARM_UTILS_BASE_CONTACT_LISTENER_H