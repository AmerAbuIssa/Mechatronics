#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"
#include "laserprocessing.h"

//! UAV drone platform controller
class Quadcopter: public Controller
{
public:
  //Default constructor - should set all the attributes to a default value for plaform and enable use of the class.
  Quadcopter();

  ~Quadcopter();

  /**
   * @brief Run the quadcopter
   * @note As per the controller interface, this is a non-blocking call, will kick of the control loop 
   */
  void run(void);

  /**
   * @brief Set the goals for the quadcopter
   * @param goals The goals to be reached
   * @return true if all goals are reachable, false otherwise
   */
  bool setGoals(std::vector<pfms::geometry_msgs::Point> goals);

  /**
   * @brief Check if the quadcopter can reach the destination from the origin
   * @param origin The origin pose, specified as odometry for the platform  
   * @param destination The destination point for the platform
   * @param distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
   * @param time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
   * @param estimatedGoalPose The estimated goal pose when reaching goal
   * @return bool indicating the platform can reach the destination from origin supplied
   */
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                 double& distance, double& time,
                                 pfms::nav_msgs::Odometry& estimatedGoalPose);


  /**
   * @brief Computes the centre of all obstacles and reports then in world coordinates
   * @note Uses the laser processing class to get the obstacles 
   * @return Vector of obstacles in world coordinates
   */
  std::vector<pfms::geometry_msgs::Point> getObstacles(void);

private:


  /** 
   * @brief This function could be used to control the quadcopter 
   */
  void reachGoal(void);

  void sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b);

  std::shared_ptr<LaserProcessing> laserProcessingPtr_;
};

#endif // QUADCOPTER_H
