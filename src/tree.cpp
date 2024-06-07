#include "tree.h"
#include <cstdlib>
#include "utils/object_types.h"

Tree::Tree(b2World *world, int treeID, const b2Vec2 &position, bool diseased,
           bool mapped, float radius)
    : treeID(treeID), diseased(diseased), mapped(mapped), radius(radius) {
  // Create Box2D body
  b2BodyDef bodyDef;
  bodyDef.type = b2_staticBody;
  bodyDef.position = position;
  body = world->CreateBody(&bodyDef);

  // Create Box2D fixture
  b2CircleShape circleShape;
  circleShape.m_radius = radius;

  b2FixtureDef fixtureDef;
  fixtureDef.shape = &circleShape;
  // Set as sensor so the drones can 'detect' them
  // When this sensor does
  // fixtureDef.density = 1.0f;
  fixtureDef.isSensor = true;
  fixtureDef.filter.categoryBits = 0x0002;
  fixtureDef.filter.maskBits = 0x0001;
  swarm_sim::UserData *userData = new swarm_sim::UserData();
  userData->type = swarm_sim::ObjectType::Tree;
  userData->tree = this;

  fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(userData);
  body->CreateFixture(&fixtureDef);

  // Set diseased status of dree
  float infectionChance = 0.05f;
  float randomValue = static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
  this->diseased = randomValue < infectionChance ? true : diseased;
}
