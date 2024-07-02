#include "tree.h"

#include <salsa/salsa.h>

#include <cstdlib>

#include "salsa/entity/target_factory.h"

Tree::Tree(b2World *world, const b2Vec2 &position, int treeID, bool diseased,
           bool mapped, float radius)
    : Target(world, position, radius),
      diseased(diseased),
      mapped(mapped),
      radius(radius) {
  // Create the sensor for the tree Target.
  b2CircleShape shape;
  shape.m_radius = radius_;

  b2FixtureDef fixtureDef;
  fixtureDef.shape = &shape;
  fixtureDef.isSensor = true;
  auto config = swarm::CollisionManager::getCollisionConfig<Tree>();

  fixtureDef.filter.categoryBits = config.categoryBits;
  fixtureDef.filter.maskBits = config.maskBits;

  swarm::UserData *userData = new swarm::UserData();
  userData->object = this;

  fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(userData);
  body_->CreateFixture(&fixtureDef);
  id = treeID;
  id_prefix = 'T';
}

Tree::~Tree() {}