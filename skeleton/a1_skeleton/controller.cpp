#include "controller.h"
#include <cmath>

Controller::Controller() :
  tolerance_(0.5), // Default tolerance of 0.5 meters
  distanceToGoal_(0.0),
  timeToGoal_(0.0),
  goalSet_(false)
{
  // Create a single PfmsConnector instance
  pfmsConnectorPtr_ = new PfmsConnector();
}

Controller::~Controller()
{
  // Clean up the PfmsConnector instance
  if (pfmsConnectorPtr_ != nullptr) {
    delete pfmsConnectorPtr_;
    pfmsConnectorPtr_ = nullptr;
  }
}

bool Controller::reachGoal(void)
{
  if (!goalSet_) {
    return false; // Cannot reach goal if no goal has been set
  }
  
  // This is a base implementation that will be called by derived classes
  // The derived classes will handle the specific control logic
  return false;
}

bool Controller::setGoal(pfms::geometry_msgs::Point goal)
{
  // Store the goal
  goal_ = goal;
  
  // Get current odometry
  pfms::nav_msgs::Odometry currentOdo = getOdometry();
  
  // Check if the goal can be reached from current position
  bool reachable = checkOriginToDestination(currentOdo, goal_, distanceToGoal_, 
                                            timeToGoal_, estimatedGoalPose_);
  
  if (reachable) {
    goalSet_ = true;
  } else {
    goalSet_ = false;
  }
  
  return reachable;
}

double Controller::distanceToGoal(void)
{
  if (!goalSet_) {
    return -1.0; // Invalid if no goal set
  }
  
  return distanceToGoal_;
}

double Controller::timeToGoal(void)
{
  if (!goalSet_) {
    return -1.0; // Invalid if no goal set
  }
  
  return timeToGoal_;
}

bool Controller::setTolerance(double tolerance)
{
  if (tolerance <= 0.0) {
    return false; // Tolerance must be positive
  }
  
  tolerance_ = tolerance;
  return true;
}

double Controller::distanceTravelled(void)
{
  return pfmsConnectorPtr_->getDistanceTravelled();
}

double Controller::timeInMotion(void)
{
  return pfmsConnectorPtr_->getTimeInMotion();
}

pfms::nav_msgs::Odometry Controller::getOdometry(void)
{
  return pfmsConnectorPtr_->getOdometry();
}
