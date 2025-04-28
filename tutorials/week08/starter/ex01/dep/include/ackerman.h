#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"

class Ackerman: public Controller
{
public:
  //! \brief Default constructor
  //! \details This constructor initializes the platform type to ACKERMAN and creates a PfmsConnector object.
  //! \details It also sets the steering ratio, lock to lock revolutions, maximum steering angle, wheelbase and maximum break torque of the Ackerman platform
  //! \details This implements the Ackerman steering geometry and does not use the audi library directly 
  Ackerman();
  
  ~Ackerman();


  //! \brief This function controls the platform to reach the goal
  //! \details Depending on distance to goal it will either drive at max velocity or use a P controller to reach the goal
  //! \details It will use @sa MAX_TROTTLE until with @sa TOLERANCE_MAX_VELOCITY (multipleier on tolerance_) 
  //! \details and then at @sa NORMAL_THROTTLE until @sa TOLERANCE_BRAKING (multipleier on tolerance_) 
  //! \details then set throttle to zero and apply hard brakes or soft brakes (p controller) on braking until it stops 
  bool reachGoal(void);

  bool setGoal(pfms::geometry_msgs::Point goal);

  bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                 double& distance, double& time, pfms::nav_msgs::Odometry& estimatedGoalPose);

private:
  //! \brief This function computes the steering angle and distance to the goal
  //! \param steering [out] The steering angle in radians
  //! \param dist [out] The distance to the goal
  bool computeSteering(double& steering,double& dist);

private:
  unsigned int ugvSeq_; //!< The sequence number for the command sent to the Ackerman platform
  const double steering_ratio_; //!< The steering ratio of the Ackerman platform
  const double lock_to_lock_revs_; //!< The number of revolutions of the steering wheel from lock to lock
  const double max_steer_angle_; //!< The maximum steering angle of the Ackerman platform
  const double wheelbase_; //!< The wheelbase of the Ackerman platform
  const double max_break_torque_; //!< The maximum break torque of the Ackerman platform
  const double steadyStateV_; //!< The steady state velocity of the Ackerman platform
  const double deltaD_; //!< The difference between the current and previous distance to the goal
  double prevD_; //!< The previous distance to the goal

  const double MAX_VEL = 4.0; //!< The maximum velocity of the Ackerman platform
  const double TOLERANCE_MAX_VELOCITY = 10.0; //!< The tolerance multiplier for the distance to the goal where it drives at max velocity if possible
  const double TOLERANCE_BRAKING = 1.2; //!< The tolerance multiplier for the distance to the goal where it drives at max velocity if possible
  const double MAX_THROTTLE = 0.25; //!< The maximum throttle of the Ackerman platform
  const double NORMAL_THROTTLE = 0.1; //!< The normal throttle of the Ackerman platform

};

#endif // ACKERMAN_H
