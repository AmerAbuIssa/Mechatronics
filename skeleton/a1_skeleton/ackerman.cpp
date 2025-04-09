#include "ackerman.h"
#include <cmath>

Ackerman::Ackerman() : Controller()
{
  // Ackerman-specific initialization
}

bool Ackerman::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose)
{
  // Use the Audi library to check if the goal can be reached
  return aud_.checkOriginToDestination(origin, goal, distance, time, estimatedGoalPose);
}

pfms::PlatformType Ackerman::getPlatformType(void)
{
  return pfms::PlatformType::ACKERMAN;
}

bool Ackerman::reachGoal(void)
{
  if (!goalSet_) {
    return false; // Cannot reach goal if no goal has been set
  }
  
  // Get current odometry
  pfms::nav_msgs::Odometry currentOdo = getOdometry();
  
  // Initialize throttle, brake, and steering
  double throttle = DEFAULT_THROTTLE;
  double brake = 0.0;
  double steering = 0.0;
  
  // Control loop
  while (distanceToGoal_ > tolerance_) {
    // Update current odometry
    currentOdo = getOdometry();
    
    // Calculate steering using the Audi library
    steering = aud_.computeSteering(currentOdo, goal_);
    
    // Clamp steering to MAX_STEER_ANGLE
    if (steering > MAX_STEER_ANGLE) {
      steering = MAX_STEER_ANGLE;
    } else if (steering < -MAX_STEER_ANGLE) {
      steering = -MAX_STEER_ANGLE;
    }
    
    // Calculate current distance to goal
    bool reachable = checkOriginToDestination(currentOdo, goal_, distanceToGoal_, 
                                              timeToGoal_, estimatedGoalPose_);
    
    if (!reachable) {
      return false; // Goal can no longer be reached
    }
    
    // Apply brakes as we get closer to the goal
    if (distanceToGoal_ < 2.0) {
      // Gradually increase brake pressure based on distance
      brake = MAX_BRAKE_TORQUE * (1.0 - distanceToGoal_ / 2.0);
      if (brake < 0) brake = 0;
      if (brake > MAX_BRAKE_TORQUE) brake = MAX_BRAKE_TORQUE;
      
      throttle = 0.0; // Cut throttle when braking
    } else {
      brake = 0.0;
      throttle = DEFAULT_THROTTLE;
    }
    
    // Send commands to the platform
    pfmsConnectorPtr_->sendCommand(throttle, steering, brake);
    
    // If we're close enough to the goal, consider it reached
    if (distanceToGoal_ <= tolerance_) {
      // Apply full brakes to stop
      pfmsConnectorPtr_->sendCommand(0.0, 0.0, MAX_BRAKE_TORQUE);
      return true;
    }
  }
  
  return true;
}
