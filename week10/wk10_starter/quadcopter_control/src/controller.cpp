#include "controller.h"
#include <cmath>

using std::placeholders::_1;

/**
 * \brief Shared functionality/base class for platform controllers
 *
 */
Controller::Controller() :
    Node("controller"),
    goalSet_(false),    
    distance_travelled_(0),
    time_travelled_(0)
{
    // We create a node handle in derived class (as they have custom messages/topics)  
    // We still have one message we could potentialy use (odo)
    sub1_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/drone/goal", 1000, std::bind(&Controller::setGoal,this,_1));

    //! @todo Subscribe to the odometry topic and set the callback

    //We set the internal variables of time/distance for goal to zero
    goal_.time=0;
    goal_.distance=0;
};

//We would now have to sacrifice having a return value to have a setGoal
//At week 10 we do not know about services (which allow us to retrun value
//So to allow to set a goal via topic we forfit having areturn value for now
//At week 11 you can replace this with a service
//bool Controller::setGoal(geometry_msgs::Point goal) {
void Controller::setGoal(const geometry_msgs::msg::Point& msg){    
    std::lock_guard<std::mutex> lock(goalMtx_);
    goal_.location = msg;
    goalSet_=true;
}

bool Controller::setTolerance(double tolerance) {
  tolerance_ = tolerance;
  return true;
}

double Controller::distanceToGoal(void) {
    return goal_.distance;
}
double Controller::timeToGoal(void) {
    return goal_.time;
}
double Controller::distanceTravelled(void) {
    return distance_travelled_;
}
double Controller::timeInMotion(void) {
    return time_travelled_;
}

GoalStats Controller::getGoalStats(void) {
    std::lock_guard<std::mutex> lock(goalMtx_);
    return goal_;
}

bool Controller::goalReached() {
    if(!goalSet_){return false;};

    GoalStats goalStats = getGoalStats();
    geometry_msgs::msg::Pose pose = getOdometry();

    double dx = goalStats.location.x - pose.position.x;
    double dy = goalStats.location.y - pose.position.y;
    double dz = goalStats.location.z - pose.position.z;

    double distance = pow(pow(dx,2)+pow(dy,2)+pow(dz,2),0.5);
    RCLCPP_INFO(this->get_logger(), "distance: %f", distance);

    return (distance < tolerance_);
}

///////////////////////////////////////////////////////////////
//! @todo
//! TASK 3 - Callback - Add implementation here, 
//! 
//! Don;t forget to use the mutex to lock the pose_ variable




geometry_msgs::msg::Pose Controller::getOdometry(void){
    std::lock_guard<std::mutex> lock(poseMtx_);
    return pose_;
}

// We do not need this function as we are not using the platform type, 
// rather, this is done by having topic names that are specific to the platform
// 
// pfms::PlatformType Controller::getPlatformType(void){
//     return type_;
// }
