#include "tree.h"

#include <cstdlib>

#include "core/simulation.h"
#include "entity/target_factory.h"

Tree::Tree(b2World *world, const b2Vec2 &position, int treeID, bool diseased,
           bool mapped, float radius)
    : Target(world, position, radius),
      diseased(diseased),
      mapped(mapped),
      radius(radius) {
  // Create Box2D fixture
  create_fixture();
  id = treeID;
  id_prefix = 'T';
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
  auto config = swarm::CollisionManager::getCollisionConfig(typeid(Tree));

  fixtureDef.filter.categoryBits = config.categoryBits;
  fixtureDef.filter.maskBits = config.maskBits;

  swarm::UserData *userData = new swarm::UserData();
  userData->type = swarm::ObjectType::Target;
  userData->object = this;

  fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(userData);
  body_->CreateFixture(&fixtureDef);
}
