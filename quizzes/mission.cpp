#include "mission.h"

Mission::Mission(std::vector<ControllerInterface*> controllers) :
    controllers_(controllers),
    objective_(mission::Objective::BASIC),
    running_(false)
{
    platformGoals_.resize(controllers.size());
}

Mission::~Mission()
{
    running_ = false;
    
    // Give threads time to process the stop signal
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Join mission thread
    if (missionThread_.joinable()) {
        missionThread_.join();
    }
}

void Mission::setGoals(std::vector<pfms::geometry_msgs::Point> goals, pfms::PlatformType platform)
{
    std::lock_guard<std::mutex> lock(missionMutex_);
    
    for (size_t i = 0; i < controllers_.size(); i++)
    {
        if (controllers_[i]->getPlatformType() == platform)
        {
            platformGoals_[i] = goals;
            controllers_[i]->setGoals(goals);
            break;
        }
    }
}

bool Mission::run()
{
    running_ = true;
    
    // Start all controllers
    for (auto controller : controllers_)
    {
        controller->run();
    }
    
    // Start mission monitoring thread
    missionThread_ = std::thread(&Mission::missionLoop, this);
    
    return true;
}

std::vector<unsigned int> Mission::status(void)
{
    std::lock_guard<std::mutex> lock(missionMutex_);
    std::vector<unsigned int> progress;
    
    for (size_t i = 0; i < controllers_.size(); i++)
    {
        progress.push_back(calculateProgress(i));
    }
    
    return progress;
}

void Mission::setMissionObjective(mission::Objective objective)
{
    objective_ = objective;
}

std::vector<double> Mission::getDistanceTravelled()
{
    std::vector<double> distances;
    
    for (auto controller : controllers_)
    {
        distances.push_back(controller->distanceTravelled());
    }
    
    return distances;
}

std::vector<double> Mission::getTimeMoving()
{
    std::vector<double> times;
    
    for (auto controller : controllers_)
    {
        times.push_back(controller->timeTravelled());
    }
    
    return times;
}

std::vector<std::pair<int, int>> Mission::getPlatformGoalAssociation()
{
    std::vector<std::pair<int, int>> association;
    
    // For BASIC mode, simple sequential assignment
    int goalIndex = 0;
    for (size_t i = 0; i < controllers_.size(); i++)
    {
        for (size_t j = 0; j < platformGoals_[i].size(); j++)
        {
            association.push_back(std::make_pair(i, goalIndex++));
        }
    }
    
    return association;
}

void Mission::missionLoop()
{
    // Simple monitoring loop for BASIC mode
    while (running_)
    {
        bool allIdle = true;
        
        for (auto controller : controllers_)
        {
            if (controller->status() != pfms::PlatformStatus::IDLE)
            {
                allIdle = false;
                break;
            }
        }
        
        if (allIdle)
        {
            running_ = false;
            break;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

unsigned int Mission::calculateProgress(int platformIndex)
{
    if (platformIndex >= controllers_.size())
        return 0;
    
    ControllerInterface* controller = controllers_[platformIndex];
    
    // Check if idle (completed)
    if (controller->status() == pfms::PlatformStatus::IDLE)
        return 100;
    //if not idle  but has no goals, return 0
    std::vector<pfms::geometry_msgs::Point> goals = platformGoals_[platformIndex];
    if (goals.empty())
        return 0;
    
     // Calculate total planned distance
     double totalDistance = 0.0;
     pfms::geometry_msgs::Point currentPos = controller->getOdometry().position;
     
     // Add distance to first goal
     totalDistance += std::sqrt(
         std::pow(goals[0].x - currentPos.x, 2) +
         std::pow(goals[0].y - currentPos.y, 2)
     );
     
     // Add distances between goals
     for (size_t i = 1; i < goals.size(); i++)
     {
         totalDistance += std::sqrt(
             std::pow(goals[i].x - goals[i-1].x, 2) +
             std::pow(goals[i].y - goals[i-1].y, 2)
         );
     }
     
     // Get actual distance travelled
     double distanceTravelled = controller->distanceTravelled();
     
     // Calculate progress percentage
     unsigned int progress = 0;
     if (totalDistance > 0) {
         progress = (unsigned int)((distanceTravelled / totalDistance) * 100);
     }
     
     // Never report more than 100% if not IDLE
     if (progress > 100) progress = 100;
     
     return progress;
}