#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h"

class Ackerman: public Controller
{
public:
  //Default constructor should set all attributes to a default value
  Ackerman();
  
  // Override pure virtual functions from Controller
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                               pfms::geometry_msgs::Point goal,
                               double& distance,
                               double& time,
                               pfms::nav_msgs::Odometry& estimatedGoalPose) override;
  
  pfms::PlatformType getPlatformType(void) override;
  
  // Override reachGoal to implement Ackerman-specific control logic
  bool reachGoal(void) override;
  
private:
  // Ackerman-specific attributes
  Audi aud_; // Use the provided Audi library
  
  // Constants for Ackerman control
  const double DEFAULT_THROTTLE = 0.1; // Vehicle steady state speed 2.91m/s
  const double MAX_BRAKE_TORQUE = 8000.0;
  const double STEERING_RATIO = 17.3;
  const double LOCK_TO_LOCK_REVS = 3.2;
  const double MAX_STEER_ANGLE = (M_PI * LOCK_TO_LOCK_REVS / STEERING_RATIO);
  const double TRACK_WIDTH = 1.638; // meters
  const double WHEELBASE = 2.65;    // meters
  const double STEADY_STATE_SPEED = 2.91; // m/s at throttle 0.1
};

#endif // ACKERMAN_H
