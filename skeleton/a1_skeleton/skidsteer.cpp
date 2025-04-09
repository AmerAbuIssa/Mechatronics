#include "skidsteer.h"
#include <cmath>

SkidSteer::SkidSteer() : Controller()
{
  // SkidSteer-specific initialization
}

bool SkidSteer::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose)
{
  // Calculate Euclidean distance from origin to goal
  double dx = goal.x - origin.position.x;
  double dy = goal.y - origin.position.y;
  double euclideanDistance = std::sqrt(dx*dx + dy*dy);
  
  // Calculate angle to goal from current heading
  double targetAngle = std::atan2(dy, dx);
  double headingError = targetAngle - origin.yaw;
  
  // Normalize angle to [-π, π]
  while (headingError > M_PI) headingError -= 2*M_PI;
  while (headingError < -M_PI) headingError += 2*M_PI;
  
  // Time to rotate to face goal
  double rotationTime = std::fabs(headingError) / MAX_ANGULAR_VELOCITY;
  
  // Time to travel to goal after rotation
  double travelTime = euclideanDistance / MAX_LINEAR_VELOCITY;
  
  // Total time is rotation time plus travel time
  time = rotationTime + travelTime;
  
  // Total distance is Euclidean distance (since rotation happens in place)
  distance = euclideanDistance;
  
  // Estimated goal pose
  estimatedGoalPose = origin;
  estimatedGoalPose.position.x = goal.x;
  estimatedGoalPose.position.y = goal.y;
  estimatedGoalPose.yaw = targetAngle;
  
  // SkidSteer can reach any goal on a flat surface
  return true;
}

pfms::PlatformType SkidSteer::getPlatformType(void)
{
  return pfms::PlatformType::SKIDSTEER;
}

double SkidSteer::calculateHeadingError(const pfms::nav_msgs::Odometry& currentOdo, 
                                       const pfms::geometry_msgs::Point& goal)
{
  double dx = goal.x - currentOdo.position.x;
  double dy = goal.y - currentOdo.position.y;
  
  // Calculate desired heading
  double desiredHeading = std::atan2(dy, dx);
  
  // Calculate heading error
  double headingError = desiredHeading - currentOdo.yaw;
  
  // Normalize to [-π, π]
  while (headingError > M_PI) headingError -= 2*M_PI;
  while (headingError < -M_PI) headingError += 2*M_PI;
  
  return headingError;
}

double SkidSteer::calculateEuclideanDistance(const pfms::geometry_msgs::Point& point1, 
                                            const pfms::geometry_msgs::Point& point2)
{
  double dx = point2.x - point1.x;
  double dy = point2.y - point1.y;
  return std::sqrt(dx*dx + dy*dy);
}

bool SkidSteer::reachGoal(void)
{
  if (!goalSet_) {
    return false; // Cannot reach goal if no goal has been set
  }
  
  // Get current odometry
  pfms::nav_msgs::Odometry currentOdo = getOdometry();
  
  // Control loop
  while (distanceToGoal_ > tolerance_) {
    // Update current odometry
    currentOdo = getOdometry();
    
    // Calculate heading error
    double headingError = calculateHeadingError(currentOdo, goal_);
    
    // Calculate current distance to goal
    distanceToGoal_ = calculateEuclideanDistance(currentOdo.position, goal_);
    
    // Update time to goal (linear velocity plus angular velocity)
    double rotationTime = std::fabs(headingError) / MAX_ANGULAR_VELOCITY;
    double travelTime = distanceToGoal_ / MAX_LINEAR_VELOCITY;
    timeToGoal_ = rotationTime + travelTime;
    
    // Initialize linear and angular velocity
    double linearVel = 0.0;
    double angularVel = 0.0;
    
    // Simple P controller for angular velocity
    const double P_GAIN_ANGULAR = 0.5;
    angularVel = P_GAIN_ANGULAR * headingError;
    
    // Limit angular velocity
    if (angularVel > MAX_ANGULAR_VELOCITY) {
      angularVel = MAX_ANGULAR_VELOCITY;
    } else if (angularVel < -MAX_ANGULAR_VELOCITY) {
      angularVel = -MAX_ANGULAR_VELOCITY;
    }
    
    // Only move forward if heading error is small
    if (std::fabs(headingError) < 0.2) {
      // Simple P controller for linear velocity
      const double P_GAIN_LINEAR = 0.5;
      linearVel = P_GAIN_LINEAR * distanceToGoal_;
      
      // Limit linear velocity
      if (linearVel > MAX_LINEAR_VELOCITY) {
        linearVel = MAX_LINEAR_VELOCITY;
      }
      
      // Reduce speed as we approach the goal
      if (distanceToGoal_ < 1.0) {
        linearVel *= distanceToGoal_;
      }
    }
    
    // Send commands to the platform
    pfms::commands::SkidSteer cmd;
    cmd.linear.x = linearVel;
    cmd.angular.z = angularVel;
    pfmsConnectorPtr_->sendCommand(cmd);
    
    // If we're close enough to the goal, consider it reached
    if (distanceToGoal_ <= tolerance_) {
      // Stop the platform
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      pfmsConnectorPtr_->sendCommand(cmd);
      return true;
    }
  }
  
  return true;
}
