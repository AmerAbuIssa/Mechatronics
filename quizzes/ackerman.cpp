#include "ackerman.h"
#include <chrono>

Ackerman::Ackerman() : Controller()
{
    pfmsConnectorPtr_ = std::make_shared<PfmsConnector>(pfms::PlatformType::ACKERMAN);
    audiLib_ = std::make_unique<Audi>();
}

pfms::PlatformType Ackerman::getPlatformType(void)
{
    return pfms::PlatformType::ACKERMAN;
}

void Ackerman::controlLoop()
{
    auto lastTime = std::chrono::steady_clock::now();
    pfms::nav_msgs::Odometry lastOdo;
    bool firstRead = true;
    distanceTravelled_ = 0.0;
    timeTravelled_ = 0.0;
    
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
                    std::pow(currentOdo.position.y - lastOdo.position.y, 2)
                );
                distanceTravelled_ += dist;
                
                // Calculate time travelled if platform is moving
                if (std::abs(currentOdo.linear.x) > 0.01) // Only count as moving if actual velocity
                {
                    auto currentTime = std::chrono::steady_clock::now();
                    timeTravelled_ += std::chrono::duration<double>(currentTime - lastTime).count();
                }
            }
            
            odometry_ = currentOdo;
            lastOdo = currentOdo;
            lastTime = std::chrono::steady_clock::now();
            firstRead = false;
            
            // Check if all goals completed BEFORE navigation
            if (!goals_.empty() && currentGoalIndex_ >= goals_.size())
            {
                std::lock_guard<std::mutex> statusLock(statusMutex_);
                status_ = pfms::PlatformStatus::IDLE;
                
                // Stop the vehicle
                pfms::commands::Ackerman stopCmd;
                stopCmd.seq = ++seq_;
                stopCmd.throttle = 0.0;
                stopCmd.brake = 8000;
                stopCmd.steering = 0.0;
                pfmsConnectorPtr_->send(stopCmd);
                
                // Continue running but don't navigate
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;  // IMPORTANT: Don't break, continue running
            }
            
            // Check if at current goal
            if (goals_.size() > 0 && currentGoalIndex_ < goals_.size() && isAtGoal())
            {
                currentGoalIndex_++;
                
                // Check if we've now completed all goals
                if (currentGoalIndex_ >= goals_.size())
                {
                    std::lock_guard<std::mutex> statusLock(statusMutex_);
                    status_ = pfms::PlatformStatus::IDLE;
                    
                    // Stop the vehicle
                    pfms::commands::Ackerman stopCmd;
                    stopCmd.seq = ++seq_;
                    stopCmd.throttle = 0.0;
                    stopCmd.brake = 8000;
                    stopCmd.steering = 0.0;
                    pfmsConnectorPtr_->send(stopCmd);
                    
                    // Continue running but don't navigate
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;  // IMPORTANT: Don't break, continue running
                }
            }
            
            // Navigate to current goal
            if (goals_.size() > 0 && currentGoalIndex_ < goals_.size())
            {
                navigateToGoal();
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

bool Ackerman::isAtGoal()
{
    if (currentGoalIndex_ >= goals_.size()) return true;
    
    pfms::geometry_msgs::Point currentGoal = goals_[currentGoalIndex_];
    double distance = std::sqrt(
        std::pow(currentGoal.x - odometry_.position.x, 2) +
        std::pow(currentGoal.y - odometry_.position.y, 2)
    );
    
    return distance < tolerance_;
}

void Ackerman::navigateToGoal()
{
    if (currentGoalIndex_ >= goals_.size()) return;
    
    pfms::geometry_msgs::Point currentGoal = goals_[currentGoalIndex_];
    
    // Use Audi library for proper steering calculation
    double distance;
    double steering;
    bool OK = audiLib_->computeSteering(odometry_, currentGoal, steering, distance);
    
    if (!OK) {
        // Goal is not reachable, stop
        pfms::commands::Ackerman stopCmd;
        stopCmd.seq = ++seq_;
        stopCmd.throttle = 0.0;
        stopCmd.brake = 8000;
        stopCmd.steering = 0.0;
        pfmsConnectorPtr_->send(stopCmd);
        return;
    }
    
    // Adjust throttle based on distance and current velocity
    double throttle = 0.2; // Base throttle
    
    // Reduce throttle when close to goal
    if (distance < 5.0) {
        throttle *= (distance / 5.0);
    }
    
    // Reduce throttle when steering is large
    if (std::abs(steering) > 0.5) {
        throttle *= 0.5;
    }
    
    // Send command
    pfms::commands::Ackerman cmd;
    cmd.seq = ++seq_;
    cmd.steering = steering;
    cmd.throttle = throttle;
    cmd.brake = 0.0;
    
    pfmsConnectorPtr_->send(cmd);
}