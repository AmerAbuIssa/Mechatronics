#include "mission.h"
#include <algorithm>
#include <limits>

Mission::Mission(std::vector<ControllerInterface*> controllers) :
  controllers_(controllers),
  objective_(mission::DISTANCE) // Default objective is DISTANCE
{
  // Nothing else to initialize
}

void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals)
{
  goals_ = goals;
  
  // Reset platform-goal association whenever goals are set
  platformGoalAssociation_.clear();
  
  // Assign goals to platforms based on objective
  assignGoalsToPlatforms();
}

bool Mission::runMission()
{
  if (goals_.empty() || controllers_.empty() || platformGoalAssociation_.empty()) {
    return false; // Cannot run mission without goals, controllers, or assignments
  }
  
  bool missionComplete = true;
  
  // For each goal
  for (size_t i = 0; i < goals_.size(); i++) {
    // Get the platform assigned to this goal
    unsigned int platformIndex = platformGoalAssociation_[i];
    
    // Make sure platform index is valid
    if (platformIndex >= controllers_.size()) {
      return false; // Invalid platform index
    }
    
    // Set the goal for the platform
    bool goalReachable = controllers_[platformIndex]->setGoal(goals_[i]);
    
    if (!goalReachable) {
      return false; // Goal cannot be reached
    }
    
    // Reach the goal
    bool goalReached = controllers_[platformIndex]->reachGoal();
    
    if (!goalReached) {
      missionComplete = false; // Goal could not be reached
    }
  }
  
  return missionComplete;
}

void Mission::setMissionObjective(mission::Objective objective)
{
  objective_ = objective;
  
  // Re-assign goals based on new objective
  assignGoalsToPlatforms();
}

std::vector<double> Mission::getDistanceTravelled()
{
  std::vector<double> distances;
  
  for (auto controller : controllers_) {
    distances.push_back(controller->distanceTravelled());
  }
  
  return distances;
}

std::vector<double> Mission::getTimeMoving()
{
  std::vector<double> times;
  
  for (auto controller : controllers_) {
    times.push_back(controller->timeInMotion());
  }
  
  return times;
}

std::vector<unsigned int> Mission::getPlatformGoalAssociation()
{
  // If association is empty, calculate it
  if (platformGoalAssociation_.empty()) {
    assignGoalsToPlatforms();
  }
  
  return platformGoalAssociation_;
}

bool Mission::assignGoalsToPlatforms()
{
  // Clear any existing assignments
  platformGoalAssociation_.clear();
  
  // If we have no goals or controllers, return false
  if (goals_.empty() || controllers_.empty()) {
    return false;
  }
  
  // If we only have one controller, assign all goals to it
  if (controllers_.size() == 1) {
    for (size_t i = 0; i < goals_.size(); i++) {
      platformGoalAssociation_.push_back(0);
    }
    return true;
  }
  
  // For simplicity in this assignment (with only 2 platforms),
  // we'll assign goals to minimize overall distance or time
  if (objective_ == mission::DISTANCE) {
    // Assign each goal to the platform that can reach it with the shortest distance
    for (const auto& goal : goals_) {
      double minDistance = std::numeric_limits<double>::max();
      unsigned int bestPlatform = 0;
      
      for (size_t i = 0; i < controllers_.size(); i++) {
        pfms::nav_msgs::Odometry currentOdo = controllers_[i]->getOdometry();
        double distance, time;
        pfms::nav_msgs::Odometry estimatedGoalPose;
        
        bool reachable = controllers_[i]->checkOriginToDestination(
          currentOdo, goal, distance, time, estimatedGoalPose);
        
        if (reachable && distance < minDistance) {
          minDistance = distance;
          bestPlatform = i;
        }
      }
      
      platformGoalAssociation_.push_back(bestPlatform);
    }
  } else { // mission::TIME
    // Assign each goal to the platform that can reach it in the shortest time
    for (const auto& goal : goals_) {
      double minTime = std::numeric_limits<double>::max();
      unsigned int bestPlatform = 0;
      
      for (size_t i = 0; i < controllers_.size(); i++) {
        pfms::nav_msgs::Odometry currentOdo = controllers_[i]->getOdometry();
        double distance, time;
        pfms::nav_msgs::Odometry estimatedGoalPose;
        
        bool reachable = controllers_[i]->checkOriginToDestination(
          currentOdo, goal, distance, time, estimatedGoalPose);
        
        if (reachable && time < minTime) {
          minTime = time;
          bestPlatform = i;
        }
      }
      
      platformGoalAssociation_.push_back(bestPlatform);
    }
  }
  
  return true;
}
