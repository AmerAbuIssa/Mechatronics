#include "controller.h"
#include <cmath>

#define DEBUG 1

///////////////////////////////////////////////////////////////
//! @todo TASK 1 - Initialisation
//!
//! Is there anything we need to initialise in the Constructor?

Controller::Controller()
{

};


pfms::PlatformType Controller::getPlatformType(void){
    return type_;
}

double Controller::distanceToGoal(void) {
    return -1;
}
double Controller::timeToGoal(void) {
    return -1;
}

bool Controller::setTolerance(double tolerance) {
  return false;
}

double Controller::distanceTravelled(void) {
    return distanceTravelled_;
}
double Controller::timeTravelled(void) {
    return timeInMotion_;
}

pfms::nav_msgs::Odometry Controller::getOdometry(void){
    pfms::nav_msgs::Odometry odo;
    pfmsConnectorPtr_->read(odo);
    return odo;
}


pfms::PlatformStatus Controller::status(void){
    return status_;
}
