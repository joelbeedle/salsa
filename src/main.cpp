#include <box2d/b2_body.h>
#include <box2d/b2_math.h>
#include <box2d/b2_polygon_shape.h>
#include <box2d/b2_time_step.h>
#include <box2d/box2d.h>

#include <SFML/Graphics.hpp>
#include <cmath>
#include <cstdio>
#include <ctime>

#include "Drone.h"
#include "DroneFactory.h"
#include "FlockingBehaviour.h"
#include "SFML/Graphics/Color.hpp"

#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600
#define MAX_FORCE 0.3f
#define VIEW_ANGLE 90.0f * (b2_pi * 180.0f)

void renderDrones(sf::RenderWindow &window, b2Body *body, float radius) {
  // Calculate the angle of the velocity
  b2Vec2 velocity = body->GetLinearVelocity();
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

void renderObstacle(sf::RenderWindow &window, b2Body *obstacle) {
  b2Vec2 position = obstacle->GetPosition();
  b2PolygonShape *shape =
      (b2PolygonShape *)obstacle->GetFixtureList()->GetShape();
  sf::ConvexShape convex;
  convex.setPointCount(shape->m_count);
  for (int i = 0; i < shape->m_count; i++) {
    b2Vec2 vertex = shape->m_vertices[i];
    convex.setPoint(i,
                    sf::Vector2f(position.x + vertex.x, position.y + vertex.y));
  }
  convex.setFillColor(sf::Color::White);
  printf("Obstacle: x: %f, y: %f, points: %d", position.x, position.y,
         shape->m_count);
  window.draw(convex);
}

void renderObstacles(sf::RenderWindow &window,
                     std::vector<b2Body *> obstacles) {
  for (auto &obstacle : obstacles) {
    renderObstacle(window, obstacle);
  }
}
void createScreenBounds(b2World *world, float screenWidth, float screenHeight) {
  // Define the ground body.
  b2BodyDef groundBodyDef;
  groundBodyDef.position.Set(0.0f, 0.0f);

  // Call the body factory which allocates memory for the ground body
  // from a pool and creates the ground box shape (also from a pool).
  // The body is also added to the world.
  b2Body *groundBody = world->CreateBody(&groundBodyDef);

  // Define the ground box shape.
  b2EdgeShape groundBox;

  // bottom
  groundBox.SetTwoSided(b2Vec2(0.0f, 0.0f), b2Vec2(screenWidth, 0.0f));
  groundBody->CreateFixture(&groundBox, 0.0f);

  // top
  groundBox.SetTwoSided(b2Vec2(0.0f, screenHeight),
                        b2Vec2(screenWidth, screenHeight));
  groundBody->CreateFixture(&groundBox, 0.0f);

  // left
  groundBox.SetTwoSided(b2Vec2(0.0f, 0.0f), b2Vec2(0.0f, screenHeight));
  groundBody->CreateFixture(&groundBox, 0.0f);

  // right
  groundBox.SetTwoSided(b2Vec2(screenWidth, 0.0f),
                        b2Vec2(screenWidth, screenHeight));
  groundBody->CreateFixture(&groundBox, 0.0f);
}

int main() {
  srand(time(NULL));
  // Create SFML window
  sf::RenderWindow window(sf::VideoMode({1200, 800}), "Drone Swarm Simulation");

  // Setup Box2d world
  b2Vec2 gravity(0.0f, 0.0f);
  b2World world(gravity);

  // Create borders
  createScreenBounds(&world, 800, 600);

  std::vector<b2Body *> obstacles;
  b2Body *currentBody = world.GetBodyList();
  while (currentBody) {
    obstacles.push_back(currentBody);
    currentBody = currentBody->GetNext();
  }

  float separationDistance = 50.0f;
  float alignmentWeight = 1.0f;
  float viewRange = 100.0f;
  float cohesionWeight = 1.0f;
  float separationWeight = 1.0f;
  float obstacleAvoidanceWeight = 1.0f;
  float maxSpeed = 10.0f;
  float maxForce = 0.3f;

  FlockingParameters params = {50.0f, 1.0f, 1.0f, 1.0f, 1.0f};
  DroneConfiguration djiMatrice300RTK_c{8.0f,  40.0f, 17.0f,  0.3f,
                                        0.45f, 6.3f,  2000.0f};

  // Create Behaviour instances
  FlockingBehaviour flockingBehaviour(params);

  // Create drones
  std::vector<std::unique_ptr<Drone>> drones;
  for (int i = 0; i < 50; i++) {
    drones.push_back(DroneFactory::createDrone(
        &world, b2Vec2(rand() % SCREEN_WIDTH, rand() % SCREEN_HEIGHT),
        flockingBehaviour, djiMatrice300RTK_c));
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
      drone->update(drones);
      renderDrones(window, drone->getBody(), 5.0f);
      // drone->render(window);
    }
    renderObstacles(window, obstacles);

    // Update Box2D world
    world.Step(1.0f / 60.0f, 6, 2);

    window.display();
  }
  return 0;
}
