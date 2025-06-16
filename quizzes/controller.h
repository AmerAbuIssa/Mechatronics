#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>
#include <pfmsconnector.h>
#include <thread>
#include <mutex>
#include <atomic>

/**
 * @file controller.h
 * @brief Abstract base class for platform controllers implementing threading and synchronization
 */

/**
 * @class Controller
 * @brief Base class providing threaded control for Mechatronic platforms
 * 
 * This class implements core functionality for managing platforms in separate threads
 * with proper synchronization mechanisms. It provides thread-safe access to shared
 * data and handles goal-based navigation.
 */
class Controller: public ControllerInterface
{
public:
    /**
     * @brief Default constructor initializing controller state
     */
    Controller();
    
    /**
     * @brief Virtual destructor ensuring proper cleanup
     */
    virtual ~Controller();
    
    /**
     * @brief Starts the control thread (non-blocking)
     * @note This method returns immediately and delegates control to a separate thread
     */
    void run(void) override;
    
    /**
     * @brief Returns current platform status
     * @return Platform status (IDLE/RUNNING)
     * @note Thread-safe operation using mutex protection
     */
    pfms::PlatformStatus status(void) override;
    
    /**
     * @brief Sets goals for the platform to navigate
     * @param goals Vector of 3D points representing navigation goals
     * @return True if goals are valid and set successfully
     * @note Thread-safe operation using mutex protection
     */
    bool setGoals(std::vector<pfms::geometry_msgs::Point> goals) override;
    
    /**
     * @brief Checks if platform can reach destination from origin
     * @param origin Starting odometry
     * @param goal Target point
     * @param distance Output: distance to travel
     * @param time Output: estimated travel time
     * @param estimatedGoalPose Output: estimated pose at goal
     * @return True if destination is reachable
     */
    bool checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                  pfms::geometry_msgs::Point goal,
                                  double& distance,
                                  double& time,
                                  pfms::nav_msgs::Odometry& estimatedGoalPose) override;
    
    /**
     * @brief Gets the platform type (must be implemented by derived classes)
     * @return Platform type identifier
     */
    pfms::PlatformType getPlatformType(void) override = 0;
    
    /**
     * @brief Returns distance to current goal
     * @return Distance in meters
     */
    double distanceToGoal(void) override;
    
    /**
     * @brief Estimates time to reach current goal
     * @return Time in seconds
     */
    double timeToGoal(void) override;
    
    /**
     * @brief Sets tolerance for goal reaching
     * @param tolerance Distance threshold for considering goal reached
     * @return True if tolerance set successfully
     */
    bool setTolerance(double tolerance) override;
    
    /**
     * @brief Returns total distance traveled
     * @return Distance in meters
     */
    double distanceTravelled(void) override;
    
    /**
     * @brief Returns total time in motion
     * @return Time in seconds
     */
    double timeTravelled(void) override;
    
    /**
     * @brief Gets current odometry data
     * @return Current platform odometry
     * @note Thread-safe operation using mutex protection
     */
    pfms::nav_msgs::Odometry getOdometry(void) override;
    
    /**
     * @brief Returns detected obstacles (empty for BASIC mode)
     * @return Vector of obstacle points
     */
    std::vector<pfms::geometry_msgs::Point> getObstacles(void) override;

protected:
    /**
     * @brief Main control loop run in separate thread (must be implemented by derived classes)
     * @note This method runs continuously until running_ flag is set to false
     */
    virtual void controlLoop() = 0;
    
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr_; //!< Connection to platform
    std::thread controlThread_;                       //!< Control thread object
    std::atomic<bool> running_;                       //!< Thread control flag
    std::mutex statusMutex_;                          //!< Protects status updates
    std::mutex goalsMutex_;                           //!< Protects goal data
    std::mutex odometryMutex_;                        //!< Protects odometry data
    
    pfms::PlatformStatus status_;                     //!< Current platform status
    std::vector<pfms::geometry_msgs::Point> goals_;   //!< Navigation goals
    unsigned int currentGoalIndex_;                   //!< Index of current goal
    double tolerance_;                                //!< Goal reaching tolerance
    double distanceTravelled_;                        //!< Total distance traveled
    double timeTravelled_;                            //!< Total time in motion
    double totalDistance_;                            //!< Total planned distance
    unsigned int seq_;                                //!< Command sequence number
    pfms::nav_msgs::Odometry odometry_;              //!< Current odometry data
    double startTime_;                                //!< Mission start time
};

#endif // CONTROLLER_H