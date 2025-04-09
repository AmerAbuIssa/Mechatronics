#ifndef SKIDSTEER_H
#define SKIDSTEER_H

#include "controller.h"

class SkidSteer: public Controller
{
public:
  //Default constructor should set all attributes to a default value
  SkidSteer();
  
  // Override pure virtual functions from Controller
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                               pfms::geometry_msgs::Point goal,
                               double& distance,
                               double& time,
                               pfms::nav_msgs::Odometry& estimatedGoalPose) override;
  
  pfms::PlatformType getPlatformType(void) override;
  
  // Override reachGoal to implement SkidSteer-specific control logic
  bool reachGoal(void) override;
  
private:
  // SkidSteer-specific attributes
  const double MAX_LINEAR_VELOCITY = 1.0;  // m/s
  const double MAX_ANGULAR_VELOCITY = 1.0; // rad/s
  
  // Helper function to calculate the angle between current heading and goal
  double calculateHeadingError(const pfms::nav_msgs::Odometry& currentOdo, 
                              const pfms::geometry_msgs::Point& goal);
                              
  // Helper function to calculate Euclidean distance
  double calculateEuclideanDistance(const pfms::geometry_msgs::Point& point1, 
                                   const pfms::geometry_msgs::Point& point2);
};

#endif // SKIDSTEER_H
