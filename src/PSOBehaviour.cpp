#include "PSOBehaviour.h"

#include <unordered_set>

#include "Drone.h"
#include "RayCastCallback.h"

float PSOBehaviour::randomFactor() {
  return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}

void PSOBehaviour::updatePersonalBest(Drone *drone) {
  int currentScore = calculateScoreForDrone(drone);

  // Update personal best if the current score is higher than the drone's
  // personal best
  if (currentScore > personalBestScores[drone]) {
    personalBestScores[drone] = currentScore;
    personalBestPositions[drone] = drone->getPosition();
  }

  // Update global best if the current score is higher than the global best
  // score
  if (currentScore > globalBestScore) {
    globalBestScore = currentScore;
    globalBestPosition = drone->getPosition();
  }
}

void PSOBehaviour::execute(std::vector<std::unique_ptr<Drone>> &drones,
                           Drone *currentDrone) {
  // Ensure personal best is initialized for the current drone
  if (personalBestPositions.find(currentDrone) == personalBestPositions.end()) {
    personalBestPositions[currentDrone] = currentDrone->getPosition();
    personalBestScores[currentDrone] = 0;
  }

  // Update personal best position based on a new metric, such as improved area
  // coverage or finding diseased trees
  updatePersonalBest(currentDrone);

  // Calculate new velocity based on PSO formula
  b2Vec2 velocity = currentDrone->getVelocity();
  b2Vec2 toPersonalBest =
      personalBestPositions[currentDrone] - currentDrone->getPosition();
  b2Vec2 toGlobalBest = globalBestPosition - currentDrone->getPosition();

  // Apply PSO formula with adjustments for b2Vec2 operations
  velocity.x *= params.inertiaWeight;
  velocity.y *= params.inertiaWeight;

  float randCognitive = params.cognitiveComponent * randomFactor();
  float randSocial = params.socialComponent * randomFactor();

  velocity.x += toPersonalBest.x * randCognitive + toGlobalBest.x * randSocial;
  velocity.y += toPersonalBest.y * randCognitive + toGlobalBest.y * randSocial;

  // Clamp the velocity to the max speed
  clampMagnitude(velocity, currentDrone->getMaxSpeed());

  // Set the new velocity
  currentDrone->getBody()->SetLinearVelocity(velocity);
}

int PSOBehaviour::calculateScoreForDrone(Drone *drone) {
  int score = 0;
  const int baseScorePerTree = 1;
  const int diseasedTreeBonus = 5;

  std::vector<Tree *> scannedTrees = drone->getFoundTrees();

  std::unordered_set<Tree *> uniqueScannedTrees(scannedTrees.begin(),
                                                scannedTrees.end());

  for (Tree *tree : uniqueScannedTrees) {
    score += baseScorePerTree;

    if (tree->isDiseased()) {
      score += diseasedTreeBonus;
    }
  }

  return score;
}