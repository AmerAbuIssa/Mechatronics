#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>
#include <pfmsconnector.h>

//! Information about the current goal for the platform
struct GoalStats {
    //! location of goal
    pfms::geometry_msgs::Point location;
    //! distance to goal
    double distance;
    //! time to goal
    double time;
};

/**
 * \brief Shared functionality/base class for platform controllers
 */
class Controller: public ControllerInterface
{
public:
  /**
   * Default Controller constructor, sets odometry and metrics to initial 0
   */
  Controller();

   //See controllerinterface.h for more information

   /** 
    * Run controller in reaching goals - non blocking call, delegated to be implemented by the platform specific controller
    */
   virtual void run(void) = 0;

    /**
      * Returns platform status (indicating if it is executing a series of goals (RUNNING) or idle (IDLE) - waiting for goals)
      * @return platform status
      */
   virtual pfms::PlatformStatus status(void);
 
    /**
      * Setter for goals, delegated to be implemented by the platform specific controller
      */
   virtual bool setGoals(std::vector<pfms::geometry_msgs::Point> goals) = 0;
 

  /** 
   * Checks whether the platform can travel between origin and destination
   * Delegated to be implemented by the platform specific controller
   */
   virtual bool checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
                                    double& distance, double& time,  pfms::nav_msgs::Odometry& estimatedGoalPose) = 0;
 
    /**
      * Getter for platform type
      * @return PlatformType
      */                                    
   pfms::PlatformType getPlatformType(void);
 
    /**
      * Getter for distance to be travelled to reach current goal
      * @return distance to be travlled to reach current goal [m]
      */
   double distanceToGoal(void);
 
    /**
      * Getter for time to reach current goal
      * @return time to travel to current goal [s]
      */
   double timeToGoal(void);
 
    /**
      * Setter for tolerance
      * @param tolerance The tolerance [m] when reaching the goal
      * @return tolerance (no negative tolerance possible)
      */
   bool setTolerance(double tolerance);
 
    /**
      * Getter for distance travelled to reach current goal
      * @return distance to be travlled to reach current goal [m]
      */
   double distanceTravelled(void);

    /**
      * Getter for time travelled to reach current goal
      * @return time to travel to current goal [s]
      */
   double timeTravelled(void);
 
    /**
      * Getter for the current odometry of the platform
      * @return current odometry of the platform
      */
   pfms::nav_msgs::Odometry getOdometry(void);

    /**
      * Getter for the current obstacles
      * delegated to be implemented by the platform specific controller
      */
   std::vector<pfms::geometry_msgs::Point> getObstacles(void) =0;

protected:


  std::shared_ptr<PfmsConnector> pfmsConnectorPtr_; //!< Pointer to the pfms connector for communication with the platform and simulator
  pfms::nav_msgs::Odometry odo_;//!< The current pose of platform

  //stats 
  std::vector<GoalStats> goals_; //!< The goals to be reached, we store all GoalStats in this vector

  pfms::PlatformType type_;
  pfms::PlatformStatus status_; //!< The current status of the platform

  double distanceTravelled_; //!< Total distance travelled for this program run
  double timeInMotion_; //!< Total time spent travelling for this program run
  double tolerance_; //!< Radius of tolerance
  long unsigned int seq_; //!<The sequence number of the command

  //threading
  bool execute_;
  std::thread* thread_;
  std::mutex mtx_; //!< Mutex for control
  std::condition_variable cv_;

};

#endif // CONTROLLER_H
