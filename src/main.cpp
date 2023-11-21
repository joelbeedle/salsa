#include "SFML/Graphics/Color.hpp"
#include <SFML/Graphics.hpp>
#include <box2d/b2_body.h>
#include <box2d/b2_math.h>
#include <box2d/b2_polygon_shape.h>
#include <box2d/b2_time_step.h>
#include <box2d/box2d.h>
#include <cmath>
#include <cstdio>
#include <ctime>

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600
#define MAX_FORCE 0.3f
#define VIEW_ANGLE 90.0f * (b2_pi * 180.0f)

class Drone {
public:
  b2Body *body;
  static constexpr float radius = 5.0f;
  b2Vec2 velocity;
  b2Vec2 acceleration;
  float perception;
  float maxSpeed;

  Drone(b2World &world, float x, float y, float perception, float maxSpeed) {
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position.Set(x, y);
    body = world.CreateBody(&bodyDef);
    double xVel = ((double)rand() / RAND_MAX) * 5.0f;
    double yVel = ((double)rand() / RAND_MAX) * 5.0f;
    velocity.Set(xVel - 1.0, yVel - 1.0);
    std::printf("xVel: %f, yVel: %f\n", xVel, yVel);

    double xAcc = ((double)rand() / RAND_MAX) / 10.0f;
    double yAcc = ((double)rand() / RAND_MAX) / 10.0f;
    acceleration.Set(xAcc, yAcc);
    std::printf("xAcc: %f, yAcc: %f\n", xAcc, yAcc);

    b2CircleShape circleShape;
    circleShape.m_radius = radius;

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &circleShape;
    fixtureDef.density = 1.0f;
    body->CreateFixture(&fixtureDef);

    this->perception = perception;
    this->maxSpeed = maxSpeed;
  }

  float angleBetween(const b2Vec2 &v1, const b2Vec2 &v2) {
    if (v1.LengthSquared() == 0.0f || v2.LengthSquared() == 0.0f) {
      return 0.0f;
    }
    float dotProduct = b2Dot(v1, v2);
    float magnitudeProduct = v1.Length() * v2.Length();
    // Clamp the value to avoid numerical issues with acos
    float cosAngle =
        std::max(-1.0f, std::min(1.0f, dotProduct / magnitudeProduct));
    return std::acos(cosAngle);
  }

  void draw(sf::RenderWindow &window) {
    // Calculate the angle of the velocity
    float angle = std::atan2(velocity.y, velocity.x);

    // Create a triangle shape
    sf::ConvexShape triangle;
    triangle.setPointCount(3);

    // Define the points of the triangle
    triangle.setPoint(
        0, sf::Vector2f(radius * std::cos(angle), radius * std::sin(angle)));
    triangle.setPoint(1, sf::Vector2f(radius * std::cos(angle + 2.5f),
                                      radius * std::sin(angle + 2.5f)));
    triangle.setPoint(2, sf::Vector2f(radius * std::cos(angle - 2.5f),
                                      radius * std::sin(angle - 2.5f)));

    // Set the position and color of the triangle
    b2Vec2 position = body->GetPosition();
    triangle.setPosition({position.x, position.y});
    triangle.setFillColor(sf::Color::White);

    // Draw the triangle
    window.draw(triangle);
  }

  void update(std::vector<Drone> &drones) {
    b2Vec2 alignment = align(drones);
    b2Vec2 coherence = cohere(drones);
    b2Vec2 separation = separate(drones);

    acceleration += alignment + coherence + separation;

    velocity += acceleration;

    b2Vec2 position = body->GetPosition();
    float velocityMagnitude = velocity.Length();
    if (velocityMagnitude > maxSpeed) {
      if (velocityMagnitude > 0.0f) {
        velocity *= (1.0f / velocityMagnitude); // Normalize by multiplying with
                                                // the inverse of the magnitude
      }

      velocity *= maxSpeed;
    }

    if (position.x > SCREEN_WIDTH) {
      position.x = 0;
    } else if (position.x < 0) {
      position.x = SCREEN_WIDTH;
    }
    if (position.y > SCREEN_HEIGHT) {
      position.y = 0;
    } else if (position.y < 0) {
      position.y = SCREEN_HEIGHT;
    }
    body->SetLinearVelocity(velocity);

    body->SetTransform(position, body->GetAngle());

    acceleration.SetZero();
  }

  float calculateDistance(const Drone &otherBoid) const {
    b2Vec2 diff = b2Vec2(body->GetPosition().x, body->GetPosition().y) -
                  b2Vec2(otherBoid.body->GetPosition().x,
                         otherBoid.body->GetPosition().y);
    return std::sqrt(diff.x * diff.x + diff.y * diff.y);
  }

  b2Vec2 align(std::vector<Drone> &drones) {
    b2Vec2 steering;
    int32 neighbours = 0;
    b2Vec2 avgVec;

    steering.SetZero();
    avgVec.SetZero();

    for (auto &drone : drones) {
      if (&drone != this) {
        float distance = calculateDistance(drone);
        if (distance < perception) {
          float angle =
              angleBetween(body->GetPosition(), drone.body->GetPosition());
          if (-VIEW_ANGLE <= angle <= VIEW_ANGLE) {
            avgVec += drone.body->GetLinearVelocity();
            neighbours++;
          }
        }
      }
    }
    if (neighbours > 0) {
      avgVec.x /= neighbours;
      avgVec.y /= neighbours;
      float length = avgVec.Length();
      if (length > 0) {
        avgVec *= (1.0f / length);
        avgVec *= maxSpeed;
      }
      steering = avgVec - body->GetLinearVelocity();
    }
    return avgVec;
  }

  b2Vec2 cohere(std::vector<Drone> &drones) {
    b2Vec2 centreOfMass;
    b2Vec2 steering;
    int32 neighbours = 0;

    centreOfMass.SetZero();
    steering.SetZero();

    for (auto &drone : drones) {
      float distance = calculateDistance(drone);
      if (distance < perception) {
        float angle =
            angleBetween(body->GetPosition(), drone.body->GetPosition());
        if (-VIEW_ANGLE <= angle <= VIEW_ANGLE) {
          centreOfMass += drone.body->GetPosition();
          neighbours++;
        }
      }
    }
    if (neighbours > 0) {
      centreOfMass.x /= neighbours;
      centreOfMass.y /= neighbours;
      b2Vec2 vecToCom = centreOfMass - body->GetPosition();
      float length = vecToCom.Length();
      if (length > 0) {
        vecToCom *= (1.0f / length);
        vecToCom *= maxSpeed;
      }
      steering = vecToCom - body->GetLinearVelocity();
      length = steering.Length();
      if (length > MAX_FORCE) {
        steering *= (1.0f / length);
        steering *= MAX_FORCE;
      }
    }
    return steering;
  }

  b2Vec2 separate(std::vector<Drone> &drones) {
    b2Vec2 steering;
    b2Vec2 avgVec;
    int32 neighbours = 0;

    steering.SetZero();
    avgVec.SetZero();

    for (auto &drone : drones) {
      float distance = calculateDistance(drone);
      if ((distance < perception / 2) &&
          (body->GetPosition() != drone.body->GetPosition())) {
        float angle =
            angleBetween(body->GetPosition(), drone.body->GetPosition());
        if (-VIEW_ANGLE <= angle <= VIEW_ANGLE) {
          b2Vec2 difference = body->GetPosition() - drone.body->GetPosition();
          difference.x /= distance;
          difference.y /= distance;
          avgVec += difference;
          neighbours++;
        }
      }
    }
    if (neighbours > 0) {
      avgVec.x /= neighbours;
      avgVec.y /= neighbours;
      float length = avgVec.Length();
      if (length > 0) {
        avgVec *= (1.0f / length);
        avgVec *= maxSpeed;
        steering = avgVec - body->GetLinearVelocity();
        length = steering.Length();
        if (length > MAX_FORCE) {
          steering *= (1.0f / length);
          steering *= MAX_FORCE;
        }
      }
    }
    return steering;
  }
};

int main() {

  srand(time(NULL));
  // Create SFML window
  sf::RenderWindow window(sf::VideoMode({800u, 600u}),
                          "Drone Swarm Simulation");

  // Setup Box2d world
  b2Vec2 gravity(0.0f, 0.0f);
  b2World world(gravity);

  // Create drones
  std::vector<Drone> drones;
  for (int i = 0; i < 50; i++) {
    drones.emplace_back(world, rand() % (800 - 2 * 5) + 5,
                        rand() % (600 - 2 * 5) + 5, 75.0f, 10.0f);
  }
  int32 counter = 0;
  while (window.isOpen()) {
    counter++;
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }

    window.clear();

    // Update Drone position
    for (auto &drone : drones) {
      drone.update(drones);
      drone.draw(window);
    }

    // Update Box2D world
    world.Step(1.0f / 60.0f, 6, 2);

    window.display();
  }
  return 0;
}
