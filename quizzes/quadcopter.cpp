#include "quadcopter.h"
#include <chrono>

Quadcopter::Quadcopter() : Controller(), hasTakenOff_(false)
{
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>(pfms::PlatformType::QUADCOPTER);
}

pfms::PlatformType Quadcopter::getPlatformType(void)
{
    return pfms::PlatformType::QUADCOPTER;
}

void Quadcopter::controlLoop()
{
    auto lastTime = std::chrono::steady_clock::now();
    pfms::nav_msgs::Odometry lastOdo;
    bool firstRead = true;
    
    while (running_)
    {
        pfms::nav_msgs::Odometry currentOdo;
        bool OK = pfmsConnectorPtr_->read(currentOdo);
        
        if (OK)
        {
            std::lock_guard<std::mutex> lock(odometryMutex_);
            
            if (!firstRead)
            {
                // Calculate distance travelled
                double dist = std::sqrt(
                    std::pow(currentOdo.position.x - lastOdo.position.x, 2) +
                    std::pow(currentOdo.position.y - lastOdo.position.y, 2) +
                    std::pow(currentOdo.position.z - lastOdo.position.z, 2)
                );
                distanceTravelled_ += dist;
                
                // Calculate time travelled if platform is moving
                if (dist > 0.01) // Small threshold to consider movement
                {
                    auto currentTime = std::chrono::steady_clock::now();
                    timeTravelled_ += std::chrono::duration<double>(currentTime - lastTime).count();
                }
            }
            
            odometry_ = currentOdo;
            lastOdo = currentOdo;
            lastTime = std::chrono::steady_clock::now();
            firstRead = false;
            
            // Ensure takeoff
            if (!hasTakenOff_)
            {
                hasTakenOff_ = takeOff();
            }
            else
            {
                // Only navigate if we have goals
                if (!goals_.empty() && currentGoalIndex_ < goals_.size())
                {
                    // Check if at current goal
                    if (isAtGoal())
                    {
                        std::lock_guard<std::mutex> goalLock(goalsMutex_);
                        currentGoalIndex_++;
                        
                        if (currentGoalIndex_ >= goals_.size())
                        {
                            std::lock_guard<std::mutex> statusLock(statusMutex_);
                            status_ = pfms::PlatformStatus::IDLE;
                            
                            // Hover in place
                            pfms::commands::Quadcopter stopCmd;
                            stopCmd.seq = ++seq_;
                            stopCmd.turn_l_r = 0.0;
                            stopCmd.move_l_r = 0.0;
                            stopCmd.move_u_d = 0.0;
                            stopCmd.move_f_b = 0.0;
                            pfmsConnectorPtr_->send(stopCmd);
                            std::this_thread::sleep_for(std::chrono::milliseconds(50));  
                            continue;
                            
                        }
                    }
                    
                    // Navigate to current goal
                    navigateToGoal();
                }
                else
                {
                    // No goals - just hover
                    pfms::commands::Quadcopter hoverCmd;
                    hoverCmd.seq = ++seq_;
                    hoverCmd.turn_l_r = 0.0;
                    hoverCmd.move_l_r = 0.0;
                    hoverCmd.move_u_d = 0.0;
                    hoverCmd.move_f_b = 0.0;
                    pfmsConnectorPtr_->send(hoverCmd);
                }
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // Back to original 50ms
    }
    
    //cleanup when exiting the loop
    std::cout << "Quadcopter control loop exiting, stopping..." << std::endl;
    
    // Send stop command before exiting
    pfms::commands::Quadcopter stopCmd;
    stopCmd.seq = ++seq_;
    stopCmd.turn_l_r = 0.0;
    stopCmd.move_l_r = 0.0;
    stopCmd.move_u_d = 0.0;
    stopCmd.move_f_b = 0.0;
    
    try {
        pfmsConnectorPtr_->send(stopCmd);
    } catch (...) {
        // Ignore exceptions during cleanup
    }
}

bool Quadcopter::takeOff()
{
    // Send takeoff command if not high enough
    if (odometry_.position.z < 0.5)
    {
        pfmsConnectorPtr_->send(pfms::PlatformStatus::TAKEOFF);
        
        // Also send upward command  
        pfms::commands::Quadcopter cmd;
        cmd.seq = ++seq_;
        cmd.turn_l_r = 0.0;
        cmd.move_l_r = 0.0;
        cmd.move_u_d = 0.1;  // Back to original 0.1
        cmd.move_f_b = 0.0;
        pfmsConnectorPtr_->send(cmd);
        
        return false;
    }
    
    return true;
}

bool Quadcopter::isAtGoal()
{
    std::lock_guard<std::mutex> lock(goalsMutex_);
    if (currentGoalIndex_ >= goals_.size()) return true;
    
    pfms::geometry_msgs::Point currentGoal = goals_[currentGoalIndex_];
    double distance = std::sqrt(
        std::pow(currentGoal.x - odometry_.position.x, 2) +
        std::pow(currentGoal.y - odometry_.position.y, 2) +
        std::pow(currentGoal.z - odometry_.position.z, 2)
    );
    
    return distance < tolerance_;
}

void Quadcopter::navigateToGoal()
{
    std::lock_guard<std::mutex> lock(goalsMutex_);
    if (currentGoalIndex_ >= goals_.size()) return;
    
    pfms::geometry_msgs::Point currentGoal = goals_[currentGoalIndex_];
    
    // Calculate desired movement
    double dx = currentGoal.x - odometry_.position.x;
    double dy = currentGoal.y - odometry_.position.y;
    double dz = currentGoal.z - odometry_.position.z;
    
    // Create command
    pfms::commands::Quadcopter cmd;
    cmd.seq = ++seq_;
    cmd.turn_l_r = 0.0; // No turning for basic mode
    
    // Simple proportional control - BACK TO ORIGINAL SCALE
    double scale = 0.5;  // Back to original 0.5
    cmd.move_f_b = dx * scale;
    cmd.move_l_r = dy * scale;
    cmd.move_u_d = dz * scale;
    
    // Limit speeds - using a higher max speed
    const double MAX_SPEED = 1.0;  // Increased from 0.5 to 1.0 for faster movement
    if (cmd.move_f_b > MAX_SPEED) cmd.move_f_b = MAX_SPEED;
    if (cmd.move_f_b < -MAX_SPEED) cmd.move_f_b = -MAX_SPEED;
    if (cmd.move_l_r > MAX_SPEED) cmd.move_l_r = MAX_SPEED;
    if (cmd.move_l_r < -MAX_SPEED) cmd.move_l_r = -MAX_SPEED;
    if (cmd.move_u_d > MAX_SPEED) cmd.move_u_d = MAX_SPEED;
    if (cmd.move_u_d < -MAX_SPEED) cmd.move_u_d = -MAX_SPEED;
    
    pfmsConnectorPtr_->send(cmd);
}