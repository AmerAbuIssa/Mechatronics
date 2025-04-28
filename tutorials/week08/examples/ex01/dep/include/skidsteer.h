#ifndef SKIDSTEER_H
#define SKIDSTEER_H

#include "controller.h"

class SkidSteer: public Controller
{
public:
  //! \brief Default constructor
  //! \details This constructor sets the maximum velocity, angular velocity and tolerance of the SkidSteer platform to be 1.0 m/s , 1.0 rad/s and 0.5m respectively.
  //! \details It also initializes the platform type to SKIDSTEER and creates a PfmsConnector object.
  SkidSteer();
  ~SkidSteer();

  //! \brief This function controls the platform to reach the goal
  //! \details It will first adjust the orinetation of the platform to face the goal, turning at @sa maxOmega_ until @sa COARSE_STEER and then via P controller until @sa MIN_STEER.
  //! \details Then it will move towards the goal at @sa maxVel_ until @sa VELOCITY_CAP_D_TOLERANCE and then it will move at a lower speed via P controller until the goal is reached.
  bool reachGoal(void);

  //! \brief This function sets the goal for the SkidSteer platform and also computes the steering angle and distance to the goal so it can be retrieved
  //! \param goal The goal to be reached
  //! @sa computeSteering  
  bool setGoal(pfms::geometry_msgs::Point goal);

  //! \brief This function computes the distance and time to the goal as well as the estimated goal pose
  //! \details It uses @sa maxVel_ and @sa maxOmega_ to compute the time to the goal and the estimated goal pose, assumes all goals are reachable
  //! \details The distance to the goal is computed using the Euclidean distance formula, time is computed using the distance and the maximum velocity of the platform to turn on spot
  //! \param origin The origin pose of the platform
  //! \param goal The goal to be reached
  //! \param distance The distance to the goal
  //! \param time The time to the goal
  //! \param estimatedGoalPose The estimated goal pose of the platform
  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                 double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose);

private:
  //! \brief This function computes the steering angle and distance to the goal
  //! \param steering [out] The steering angle in radians
  //! \param dist [out] The distance to the goal
  bool computeSteering(double& steering,double& dist);

private:
  unsigned int ugvSeq_; //!< The sequence number for the command sent to the SkidSteer platform
  const double maxVel_; //!< The maximum velocity of SkidSteer platform
  const double maxOmega_; //!< The maximum angular velocity of SkidSteer platform
  double prevD_; //!< The previous distance to the goal
  double deltaD_; //!< The difference between the current and previous distance to the goal
  const double COARSE_STEER = 30.0; //!< The coarse steering angle difference that will be used to turn the platform at max angular velocity
  const double MIN_STEER = 5.0; //!< The coarse steering angle in degrees below which the platform will not turn
  const double VELOCITY_CAP_D_TOLERANCE = 0.7; //!< The distance tolerance that max velocity caps at
};

#endif // SKIDSTEER_H
