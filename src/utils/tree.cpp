#include "utils/tree.h"

#include <cstdlib>

#include "utils/collision_manager.h"
#include "utils/entity.h"
#include "utils/object_types.h"
namespace swarm {
Tree::Tree(b2World *world, int treeID, const b2Vec2 &position, bool diseased,
           bool mapped, float radius)
    : Entity(world, position, true),
      treeID(treeID),
      diseased(diseased),
      mapped(mapped),
      radius(radius) {
  // Create Box2D fixture
  create_fixture();

  // Set diseased status of dree
  float infectionChance = 0.05f;
  float randomValue =
      static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
  this->diseased = randomValue < infectionChance ? true : diseased;

  // CollisionManager::registerType(typeid(Tree), {typeid(Drone)});
}

Tree::~Tree() {}

void Tree::create_fixture() {
  b2CircleShape shape;
  shape.m_radius = radius;

  b2FixtureDef fixtureDef;
  fixtureDef.shape = &shape;
  fixtureDef.isSensor = true;
  CollisionConfig config = CollisionManager::getCollisionConfig(typeid(Tree));

  fixtureDef.filter.categoryBits = config.categoryBits;
  fixtureDef.filter.maskBits = config.maskBits;

  UserData *userData = new UserData();
  userData->type = ObjectType::Tree;
  userData->object = this;

  fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(userData);
  body_->CreateFixture(&fixtureDef);
}
}  // namespace swarm