#include "quadcopter.h"
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <time.h>

#define DEBUG 1

using std::cout;
using std::endl;

///////////////////////////////////////////////////////////////
//! @todo
//! TASK 1 - Initialisation
//!
//! Is there anything we need to initialise in the Constructor?
//!
//! We have created a pfmsConnectorPtr_ and a laserProcessingPtr_ for you here

Quadcopter::Quadcopter()    
{
    type_ = pfms::PlatformType::QUADCOPTER; //Type is quadcopter
    // We create a pointer to the PfmConnector here in the constructor, so we can OPEN connection ONLY once
    pfmsConnectorPtr_ = std::make_unique<PfmsConnector>(type_);

    // We create a pointer to the LaserProcessing object here in the constructor, so we can OPEN connection ONLY once
    laserProcessingPtr_ = std::make_unique<LaserProcessing>(pfmsConnectorPtr_);
}

Quadcopter::~Quadcopter(){
    // Stop the quadcopter immediately, sending it down
    sendCmd(0, 0, -1.0,0);     
}

///////////////////////////////////////////////////////////////
//! @todo TASK 3 - Contorol the quadcopter
//!
void Quadcopter::run(void){

    //! Run needs to return immediately, so you will definately need to coordinate another thread

}

bool Quadcopter::checkOriginToDestination(pfms::nav_msgs::Odometry origin, pfms::geometry_msgs::Point goal,
    double& distance, double& time,
    pfms::nav_msgs::Odometry& estimatedGoalPose) {

    // We need to check if the goal is reachable from the origin

    return true;
}

///////////////////////////////////////////////////////////////
//! @todo TASK 3 - Goals for the quadcopter
//!
bool Quadcopter::setGoals(std::vector<pfms::geometry_msgs::Point> goals) {

    // We need to check if all the goals are reachable here
    // You can leverage your checkOriginToDestination function to do this

    return true;
}


void Quadcopter::sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b) {
    pfms::commands::Quadcopter cmd = {
        seq_++,
        turn_l_r,
        move_l_r,
        move_u_d,
        move_f_b,
    };
    pfmsConnectorPtr_->send(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));//Small delay to ensure message sent
}

///////////////////////////////////////////////////////////////
//! @todo TASK 3 and 4 - Contorol the quadcopter
//!
void Quadcopter::reachGoal(void) {

    // We have created a function to control the quadcopter called reachGoal, 
    // This is a private function, we do not want others to call this function
    // We will use this function to control the quadcopter

}


std::vector<pfms::geometry_msgs::Point> Quadcopter::getObstacles(void) {

    std::vector<pfms::geometry_msgs::Point> obstacles;
    obstacles =  laserProcessingPtr_->getObstacles();

    //Transform the data using the odometry
    pfms::nav_msgs::Odometry odo = getOdometry();

    return obstacles;
}