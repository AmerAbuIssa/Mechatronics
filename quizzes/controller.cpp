#include "controller.h"
#include <chrono>

Controller::Controller() :
    running_(false),
    status_(pfms::PlatformStatus::IDLE),
    currentGoalIndex_(0),
    tolerance_(0.5),
    distanceTravelled_(0.0),
    timeTravelled_(0.0),
    totalDistance_(0.0),
    seq_(0),
    startTime_(0.0)
{
}

Controller::~Controller()
{
    running_ = false;
    if (controlThread_.joinable()) {
        controlThread_.join();
    }
    std::cout << "Controller destroyed safely." << std::endl;
}

void Controller::run(void)
{
    running_ = true;
    std::lock_guard<std::mutex> lock(statusMutex_);
    status_ = pfms::PlatformStatus::RUNNING;
    controlThread_ = std::thread(&Controller::controlLoop, this);
}

pfms::PlatformStatus Controller::status(void)
{
    std::lock_guard<std::mutex> lock(statusMutex_);
    return status_;
}

bool Controller::setGoals(std::vector<pfms::geometry_msgs::Point> goals)
{
    std::lock_guard<std::mutex> lock(goalsMutex_);
    goals_ = goals;
    currentGoalIndex_ = 0;
    
    // Calculate total distance
    if (!goals.empty() && pfmsConnectorPtr_->read(odometry_)) {
        pfms::geometry_msgs::Point currentPos = odometry_.position;
        totalDistance_ = 0.0;
        for (unsigned int i = 0; i < goals.size(); i++) {
            double distance = std::sqrt(
                std::pow(goals[i].x - currentPos.x, 2) +
                std::pow(goals[i].y - currentPos.y, 2) +
                std::pow(goals[i].z - currentPos.z, 2)
            );
            totalDistance_ += distance;
            currentPos = goals[i];
        }
    }
    
    return true;
}

bool Controller::checkOriginToDestination(pfms::nav_msgs::Odometry origin,
                                          pfms::geometry_msgs::Point goal,
                                          double& distance,
                                          double& time,
                                          pfms::nav_msgs::Odometry& estimatedGoalPose)
{
    distance = std::sqrt(
        std::pow(goal.x - origin.position.x, 2) +
        std::pow(goal.y - origin.position.y, 2) +
        std::pow(goal.z - origin.position.z, 2)
    );
    
    // Estimate time based on platform speed (adjust as needed)
    double estimatedSpeed = 1.0; // m/s
    time = distance / estimatedSpeed;
    
    estimatedGoalPose = origin;
    estimatedGoalPose.position = goal;
    
    return true;
}

double Controller::distanceToGoal(void)
{
    std::lock_guard<std::mutex> lock(goalsMutex_);
    if (currentGoalIndex_ >= goals_.size()) return 0.0;
    
    pfmsConnectorPtr_->read(odometry_);
    pfms::geometry_msgs::Point currentGoal = goals_[currentGoalIndex_];
    
    return std::sqrt(
        std::pow(currentGoal.x - odometry_.position.x, 2) +
        std::pow(currentGoal.y - odometry_.position.y, 2) +
        std::pow(currentGoal.z - odometry_.position.z, 2)
    );
}

double Controller::timeToGoal(void)
{
    double distance = distanceToGoal();
    double estimatedSpeed = 1.0; // m/s
    return distance / estimatedSpeed;
}

bool Controller::setTolerance(double tolerance)
{
    tolerance_ = tolerance;
    return true;
}

double Controller::distanceTravelled(void)
{
    return distanceTravelled_;
}

double Controller::timeTravelled(void)
{
    return timeTravelled_;
}

pfms::nav_msgs::Odometry Controller::getOdometry(void)
{
    std::lock_guard<std::mutex> lock(odometryMutex_);
    return odometry_;
}

std::vector<pfms::geometry_msgs::Point> Controller::getObstacles(void)
{
    return std::vector<pfms::geometry_msgs::Point>(); // Empty for BASIC mode
}