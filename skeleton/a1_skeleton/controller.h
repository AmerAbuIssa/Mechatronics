#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include "pfmsconnector.h"

class Controller: public ControllerInterface
{
public:
  //Default constructors should set all attributes to a default value
  Controller();
  ~Controller();

  // Common functionality for all controllers
  bool reachGoal(void) override;
  bool setGoal(pfms::geometry_msgs::Point goal) override;
  double distanceToGoal(void) override;
  double timeToGoal(void) override;
  bool setTolerance(double tolerance) override;
  double distanceTravelled(void) override;
  double timeInMotion(void) override;
  pfms::nav_msgs::Odometry getOdometry(void) override;
  
  // Pure virtual functions to be implemented by derived classes
  virtual bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                        pfms::geometry_msgs::Point goal,
                                        double& distance,
                                        double& time,
                                        pfms::nav_msgs::Odometry& estimatedGoalPose) override = 0;
  
  virtual pfms::PlatformType getPlatformType(void) override = 0;

protected:
  // Common attributes for all controllers
  PfmsConnector* pfmsConnectorPtr_; // Only one instance per controller
  pfms::geometry_msgs::Point goal_;
  double tolerance_;
  double distanceToGoal_;
  double timeToGoal_;
  bool goalSet_;
  pfms::nav_msgs::Odometry estimatedGoalPose_;
};

#endif // CONTROLLER_H
