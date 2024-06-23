#include "core/simulation.h"
namespace swarm {
class DSPPoint {
 public:
  float radius = 2.0f;
  float v_max = 45.0f;
  float mass = 1.0f;
  float p = 2.0f;
  float F_max = (mass * v_max) / (1.0f / 30.0f);
  // float density = (M_PI / 4);
  float density = (M_PI * sqrt(3)) / 6;
  float area = 2000.0f * 2000.0f;
  float maxAreaCoverage = density * area;
  float numAgents = 50.0f;
  float searchAreaPerAgent = maxAreaCoverage / numAgents;
  float R = (numAgents / 10.0f) * sqrt(searchAreaPerAgent / M_PI);
  float G_const = F_max * pow(R, p) * pow(2 - pow(1.5f, (1 - p)), p / (1 - p));

  b2Vec2 position;
  b2Vec2 velocity;
  b2Body *body;

  DSPPoint(b2World *world, const b2Vec2 &position) {
    b2BodyDef bodyDef;
    bodyDef.type = b2_dynamicBody;
    bodyDef.position = position;
    body = world->CreateBody(&bodyDef);

    // Create Box2D fixture
    b2CircleShape circleShape;
    circleShape.m_radius = radius;

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &circleShape;
    float area_m2 = M_PI * pow(radius, 2);

    float density_box2d = mass / area_m2;

    fixtureDef.density = density_box2d;

    body->CreateFixture(&fixtureDef);

    body->SetLinearVelocity(b2Vec2(0, 0));
  }

  float gravDSPForce(b2Vec2 &position, b2Vec2 &otherPoint) {
    float distance = b2Distance(otherPoint, position);
    if (distance < 0.0001f) {
      distance = 0.0001f;
    }

    float forceDirection =
        (distance > R) ? 1.0f
                       : -1.0f;  // Attractive if greater than R, else repulsive

    float result = forceDirection * G_const / pow(distance, p);
    return result;
  }

  void recalc(int numDrones) {
    numAgents = static_cast<float>(numDrones);
    searchAreaPerAgent = maxAreaCoverage / numAgents;
    R = (2 + (numAgents / 20.0f)) * sqrt(searchAreaPerAgent / M_PI);
    G_const = F_max * pow(R, p) * pow(2 - pow(1.5f, (1 - p)), p / (1 - p));
  }
};

class DSPBehaviour : public Behaviour {
 private:
  std::vector<DSPPoint *> dspPoints;
  float firstRun = true;

  std::unordered_map<std::string, behaviour::Parameter *> parameters_ = {};

  struct DroneInfo {
    bool isAtDSPPoint;
    b2Vec2 dspPoint;
    DSPPoint *dsp;
    bool beginWalk = false;
    float elapsedTime = 0.0f;
    // calculated from sqrt(4000000 + 4000000) / (2 * 10.0f);
    // 141.421356237f
    float timeToWalk = 141.421356237f;
    float elapsedTimeSinceLastForce = 0.0f;
    float randomTimeInterval = 1.0f;
    b2Vec2 desiredVelocity;

    DroneInfo() : randomTimeInterval(generateRandomTimeInterval()) {}
  };

  std::unordered_map<Drone *, DroneInfo> droneInformation;

 public:
  DSPBehaviour() {}

  void execute(const std::vector<std::unique_ptr<Drone>> &drones,
               Drone &currentDrone) override {
    if (droneInformation.find(&currentDrone) == droneInformation.end()) {
      droneInformation[&currentDrone] = DroneInfo();
      DSPPoint *dsp = new DSPPoint(currentDrone.getBody()->GetWorld(),
                                   currentDrone.getPosition());
      dsp->recalc(drones.size());
      droneInformation[&currentDrone].dsp = dsp;
      dspPoints.push_back(dsp);
    }
    RayCastCallback callback;
    performRayCasting(currentDrone, callback);
    std::vector<b2Body *> bodies;
    // collect DSP points from other drones
    DroneInfo &droneInfo = droneInformation[&currentDrone];

    for (auto &drone : drones) {
      bodies.push_back(drone.get()->getBody());
    }
    std::vector<b2Body *> neighbours = bodies;
    std::vector<b2Vec2> obstaclePoints = callback.obstaclePoints;
    b2Vec2 obstacleAvoidance = avoidObstacles(obstaclePoints, currentDrone);
    b2Vec2 neighbourAvoidance = avoidDrones(neighbours, currentDrone);

    b2Vec2 bspPos = droneInfo.dsp->body->GetPosition();
    b2Vec2 force(0.0f, 0.0f);
    float forceMag = 0.0f;
    b2Vec2 overallDir(0.0f, 0.0f);
    for (auto &point : dspPoints) {
      if (point != droneInfo.dsp) {
        b2Vec2 pointPos = point->body->GetPosition();
        float forceMagnitude = droneInfo.dsp->gravDSPForce(bspPos, pointPos);
        b2Vec2 direction = directionTo(bspPos, pointPos);
        direction.Normalize();
        force +=
            forceMagnitude *
            direction;  // Calculate and accumulate the correct force vector
      }
    }

    // Clamp the force if it exceeds the maximum allowable force
    if (force.Length() > 4000.0f) {
      force.Normalize();
      force *= 4000.0f;
    }

    // Update DSP position for this drone
    droneInfo.dsp->body->SetLinearVelocity(force);

    b2Vec2 velocity = currentDrone.getVelocity();
    b2Vec2 position = currentDrone.getPosition();
    b2Vec2 acceleration(0, 0);
    b2Vec2 steer(0, 0);

    float distanceToDSP = b2Distance(position, bspPos);
    if (distanceToDSP < 40.0f) {
      if (!droneInfo.beginWalk && droneInfo.elapsedTime == 0.0f) {
        // Start the random walk if not already started and timer is reset
        droneInfo.beginWalk = true;
        droneInfo.elapsedTime = 0.0f;
      }
    }
    // Handle random walk logic
    if (droneInfo.beginWalk) {
      if (droneInfo.elapsedTime >= droneInfo.timeToWalk) {
        // Walk time is up, stop walking
        droneInfo.beginWalk = false;
        droneInfo.elapsedTime = 0.0f;
      } else {
        // Continue walking
        if (droneInfo.elapsedTimeSinceLastForce >=
            droneInfo.randomTimeInterval) {
          // Change direction at regular intervals
          float angle = static_cast<float>(std::rand()) / RAND_MAX * 2 * M_PI;
          droneInfo.desiredVelocity =
              b2Vec2(std::cos(angle) * currentDrone.getMaxSpeed(),
                     std::sin(angle) * currentDrone.getMaxSpeed());

          droneInfo.elapsedTimeSinceLastForce =
              0.0f;  // Reset the timer for force update
          droneInfo.randomTimeInterval =
              generateRandomTimeInterval();  // Next interval
        }

        droneInfo.elapsedTimeSinceLastForce += (1.0 / 30.0f);
        droneInfo.elapsedTime += (1.0 / 30.0f);

        // Apply random walk velocity as steering force
        b2Vec2 steer = droneInfo.desiredVelocity - currentDrone.getVelocity();
        clampMagnitude(steer, currentDrone.getMaxForce());
        acceleration += steer;
      }
    } else {
      // If not walking, drone should head towards the DSP point
      b2Vec2 dir = directionTo(position, bspPos);
      dir.Normalize();
      b2Vec2 steering = currentDrone.getMaxSpeed() * dir;
      clampMagnitude(steering, currentDrone.getMaxForce());
      acceleration += steering;
    }
    acceleration += neighbourAvoidance + (3.0f * obstacleAvoidance);
    velocity += acceleration;
    float speed = 0.001f + velocity.Length();
    b2Vec2 dir(velocity.x / speed, velocity.y / speed);

    // Clamp speed
    if (speed > currentDrone.getMaxSpeed()) {
      speed = currentDrone.getMaxSpeed();
    } else if (speed < 0) {
      speed = 0.001f;
    }
    velocity = b2Vec2(dir.x * speed, dir.y * speed);

    currentDrone.getBody()->SetLinearVelocity(velocity);
    acceleration.SetZero();
  }

  std::unordered_map<std::string, behaviour::Parameter *> getParameters()
      override {
    return parameters_;
  }

  void clean(const std::vector<std::unique_ptr<Drone>> &drones) override {
    for (auto &point : dspPoints) {
      b2World *world = point->body->GetWorld();
      world->DestroyBody(point->body);
    }
    dspPoints.clear();
    droneInformation.clear();
  }

 private:
  b2Vec2 directionTo(b2Vec2 &position, b2Vec2 &otherPoint) {
    float angle = atan2((otherPoint.y - position.y), otherPoint.x - position.x);
    b2Vec2 direction(cos(angle), sin(angle));
    return direction;
  }
  static float generateRandomTimeInterval() {
    return static_cast<float>(std::rand()) / RAND_MAX * 15.0f;
  }
};

auto d = std::make_unique<DSPBehaviour>();
auto dsp_behaviour =
    behaviour::Registry::getInstance().add("DSPBehaviour", std::move(d));
}  // namespace swarm