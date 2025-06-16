#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"

/**
 * @file quadcopter.h
 * @brief Controller for quadcopter platform with 3D navigation
 */

/**
 * @class Quadcopter
 * @brief Platform controller for aerial quadcopter vehicle
 * 
 * This class implements control logic for the quadcopter platform, managing
 * takeoff, 3D navigation, and hovering capabilities. Handles flight commands
 * for forward/backward, left/right, up/down, and rotation.
 */
class Quadcopter: public Controller
{
public:
    /**
     * @brief Constructor initializing quadcopter controller
     */
    Quadcopter();
    
    /**
     * @brief Returns platform type identifier
     * @return QUADCOPTER platform type
     */
    pfms::PlatformType getPlatformType(void) override;
    
protected:
    /**
     * @brief Main control loop for quadcopter platform
     * @details Manages takeoff, navigation, and goal reaching
     * @note Runs in separate thread, handles 3D movement
     */
    void controlLoop() override;
    
private:
    /**
     * @brief Checks if quadcopter has reached current goal
     * @return True if within tolerance of current goal in 3D space
     */
    bool isAtGoal();
    
    /**
     * @brief Generates and sends navigation commands to reach current goal
     * @details Implements proportional control for 3D movement
     */
    void navigateToGoal();
    
    /**
     * @brief Handles takeoff sequence
     * @return True when takeoff is complete
     * @details Sends takeoff command and maintains upward movement until sufficient height
     */
    bool takeOff();
    
    bool hasTakenOff_;                //!< Flag indicating takeoff completion
    const double HOVER_HEIGHT = 2.0;  //!< Default hovering height in meters
    const double MAX_SPEED = 1.0;     //!< Maximum speed limit for safety
};

#endif // QUADCOPTER_H