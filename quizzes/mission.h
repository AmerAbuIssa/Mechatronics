#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "missioninterface.h"
#include "pfmsconnector.h"
#include <thread>
#include <mutex>
#include <atomic>

/**
 * @file mission.h
 * @brief Mission management system for coordinating multiple platforms
 */

/**
 * @class Mission
 * @brief Coordinates multiple platforms to achieve mission objectives
 * 
 * This class manages the execution of missions across multiple platforms,
 * providing synchronized control, progress tracking, and status reporting.
 * It implements thread-safe coordination of platform operations.
 */
class Mission: public MissionInterface
{
public:
    /**
     * @brief Constructor initializing mission with platform controllers
     * @param controllers Vector of platform controllers to coordinate
     */
    Mission(std::vector<ControllerInterface*> controllers);
    
    /**
     * @brief Destructor ensuring clean shutdown of mission threads
     */
    ~Mission();
    
    /**
     * @brief Assigns goals to specific platform
     * @param goals Vector of goal points
     * @param platform Platform type identifier
     * @note Thread-safe operation using mutex protection
     */
    void setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform) override;
    
    /**
     * @brief Starts mission execution (non-blocking)
     * @return True if mission started successfully
     * @note Creates monitoring thread for progress tracking
     */
    bool run() override;
    
    /**
     * @brief Returns progress status for all platforms
     * @return Vector of progress percentages (0-100)
     * @note Thread-safe operation using mutex protection
     */
    std::vector<unsigned int> status(void) override;
    
    /**
     * @brief Sets mission execution mode
     * @param objective Mission mode (BASIC/ADVANCED/SUPER)
     */
    void setMissionObjective(mission::Objective objective) override;
    
    /**
     * @brief Gets total distance traveled by each platform
     * @return Vector of distances in meters
     */
    std::vector<double> getDistanceTravelled() override;
    
    /**
     * @brief Gets total moving time for each platform
     * @return Vector of times in seconds
     */
    std::vector<double> getTimeMoving() override;
    
    /**
     * @brief Gets platform-goal associations for tracking
     * @return Vector of platform-goal index pairs
     */
    std::vector<std::pair<int, int>> getPlatformGoalAssociation() override;

private:
    /**
     * @brief Mission monitoring loop running in separate thread
     * @details Tracks all platforms until completion
     */
    void missionLoop();
    
    /**
     * @brief Calculates progress percentage for specific platform
     * @param platformIndex Index of platform in controllers vector
     * @return Progress percentage (0-100)
     * @note Progress is 100% only when platform status is IDLE
     */
    unsigned int calculateProgress(int platformIndex);
    
    std::vector<ControllerInterface*> controllers_;                           //!< Platform controllers
    std::vector<std::vector<pfms::geometry_msgs::Point>> platformGoals_;      //!< Goals per platform
    mission::Objective objective_;                                            //!< Mission mode
    std::mutex missionMutex_;                                                 //!< Protects mission state
    std::atomic<bool> running_;                                               //!< Mission execution flag
    std::thread missionThread_;                                               //!< Monitoring thread
};

#endif // MISSION_H