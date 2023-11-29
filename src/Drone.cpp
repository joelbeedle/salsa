// Drone.cpp
#include "Drone.h"

#include <valarray>

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600

Drone::Drone(b2World *world, const b2Vec2 &position, SwarmBehaviour *behaviour)
    : behaviour(behaviour), perception(100.0f), maxSpeed(5.0f) {
  // Create Box2D body
  b2BodyDef bodyDef;
  bodyDef.type = b2_dynamicBody;
  bodyDef.position = position;
  body = world->CreateBody(&bodyDef);

  // Create Box2D fixture
  b2CircleShape circleShape;
  circleShape.m_radius = 5.0f;  // TODO: use constant or parameter for radius

  b2FixtureDef fixtureDef;
  fixtureDef.shape = &circleShape;
  fixtureDef.density = 1.0f;
  body->CreateFixture(&fixtureDef);

  std::vector<b2Body *> obstacles;
}

Drone::~Drone() {
  if (body) {
    b2World *world = body->GetWorld();
    if (world) {
      world->DestroyBody(body);
    }
  }
  if (behaviour) {
    delete behaviour;
  }
}

void Drone::update(std::vector<Drone *> &drones) {
  if (behaviour) {
    behaviour->execute(drones, this);
  }

  b2Vec2 position = body->GetPosition();
  b2Vec2 velocity = body->GetLinearVelocity();

  float velocityMagnitude = velocity.Length();
  if (velocityMagnitude > maxSpeed) {
    if (velocityMagnitude > 0.0f) {
      velocity *= (1.0f / velocityMagnitude);  // Normalize by multiplying with
                                               // the inverse of the magnitude
    }

    velocity *= maxSpeed;
  }

  body->SetLinearVelocity(velocity);

  body->SetTransform(position, body->GetAngle());

  // acceleration.SetZero(); needs to be in execute()
}

// void Drone::render(sf::RenderWindow &window) {
//   // Calculate the angle of the velocity
//   b2Vec2 velocity = body->GetLinearVelocity();
//   float angle = std::atan2(velocity.y, velocity.x);

//   // Create a triangle shape
//   sf::ConvexShape triangle;
//   triangle.setPointCount(3);

//   // Define the points of the triangle
//   triangle.setPoint(
//       0, sf::Vector2f(radius * std::cos(angle), radius * std::sin(angle)));
//   triangle.setPoint(1, sf::Vector2f(radius * std::cos(angle + 2.5f),
//                                     radius * std::sin(angle + 2.5f)));
//   triangle.setPoint(2, sf::Vector2f(radius * std::cos(angle - 2.5f),
//                                     radius * std::sin(angle - 2.5f)));

//   // Set the position and color of the triangle
//   b2Vec2 position = body->GetPosition();
//   triangle.setPosition({position.x, position.y});
//   triangle.setFillColor(sf::Color::White);

//   // Draw the triangle
//   window.draw(triangle);
// }

void Drone::setBehaviour(SwarmBehaviour *newBehaviour) {
  if (behaviour) {
    delete behaviour;
  }

  behaviour = newBehaviour;
}

b2Body *Drone::getBody() { return body; }

b2Vec2 Drone::getVelocity() { return body->GetLinearVelocity(); }

b2Vec2 Drone::getPosition() { return body->GetPosition(); }