#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "controller.h"
#include "audi.h" // Include Audi library

/**
 * @file ackerman.h
 * @brief Controller for Audi R8 platform with Ackerman steering
 */

/**
 * @class Ackerman
 * @brief Platform controller for ground vehicle with Ackerman steering geometry
 * 
 * This class implements control logic for the Audi R8 platform, using the Audi
 * library for steering calculations and managing throttle, brake, and steering
 * commands.
 */
class Ackerman: public Controller
{
public:
    /**
     * @brief Constructor initializing Ackerman platform controller
     */
    Ackerman();
    
    /**
     * @brief Returns platform type identifier
     * @return ACKERMAN platform type
     */
    pfms::PlatformType getPlatformType(void) override;
    
protected:
    /**
     * @brief Main control loop for Ackerman platform
     * @details Manages navigation, odometry updates, and command execution
     * @note Runs in separate thread, handles goal reaching detection
     */
    void controlLoop() override;
    
private:
    /**
     * @brief Checks if platform has reached current goal
     * @return True if within tolerance of current goal
     */
    bool isAtGoal();
    
    /**
     * @brief Generates and sends navigation commands to reach current goal
     * @details Uses Audi library for steering calculations and implements speed control
     */
    void navigateToGoal();
    
    std::unique_ptr<Audi> audiLib_;  //!< Audi library for steering calculations
};

#endif // ACKERMAN_H