// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This executable shows the use of the pfmsconnector library to send commands
// and receive odometry from the Ackerman platform
//
// The program will drive the platform to a goal and stop when it reaches the goal
// There is a state machine to control the platform
// From IDLE to ACCELERATE to COAST to STOPPING
//
// The control here is rudimentary and can be improved, there is no P controll here
// The control is base don a threshold of tolerance, and shoudl be improved
// As Assignment 1 examines your control logic, we can not provide you with the control logic here
// You need to develop your own control logic


#include "pfms_types.h" // Acces to all the types needed for using the pfms libraries (audi and pfmsconnector)
#include "pfmsconnector.h" // Access to the PfmsConnector class - to send and receive messages from the platform
#include "audi.h" // Access to the Audi class - to compute steering and other information
#include <iostream> // Access to the cout and endl objects for printing
#include <stdlib.h> // Access to the exit function
#include <thread> // Access to the sleep_for function
#include <chrono> // Access to the milliseconds function to specify the sleep time

using std::cout;
using std::endl;

// Control has a few states
namespace control{
    enum State
    {
        IDLE,
        ACCELERATE,
        COAST,
        STOPPING
    };
}


//We can print a summary of the control structure as we created a << operator for it
std::ostream& operator<<(std::ostream& os, control::State state)
{

    switch(state)
    {
        case control::State::IDLE   : os << "IDLE ";    break;
        case control::State::ACCELERATE : os << "ACCELERATE ";  break;
        case control::State::COAST : os << "COAST ";  break;
        case control::State::STOPPING : os << "STOPPING ";  break;
    }
    return os;
}


int main(int argc, char *argv[]) {


    double x=10.0;
    double y=5.0;

    //! Let's create an instance of the Audi class
    Audi audi;

    double tolerance = 0.5;

    control::State state = control::IDLE; // We start in the IDLE state

    //! Created a pointer to a PfmsConnector object
    pfms::PlatformType type = pfms::PlatformType::ACKERMAN;
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr = std::make_shared<PfmsConnector>(type);
    
    
    pfms::nav_msgs::Odometry odo; // We will use this to store odometry

    //! Our goal is here to keep driving the Ackerman platform towards the goal, stopping at the goal withon a tolerance
    //! We will use the odometry to check if we are close to the goal
    //! We will use the Ackerman command to drive the platform
    //! We will use the pfmsconnector to send commands and receive odometry data



    //! We can also send a goal to be visualised on rviz with the following command
    unsigned int j=0; // You will need to increas this number each time you send a new goal
    pfms::geometry_msgs::Point goal{x,y}; // Our goal in appropriate format
    pfms::geometry_msgs::Goal goalViz{j++,goal}; 
    pfmsConnectorPtr->send(goalViz);

    bool goalReached=false;// Indicates if we reach the goal to stop the control
    double distance; // Used to store distance to goal
    double steering = 0.0; // Used to store steering to goal
    pfms::nav_msgs::Odometry estimatedGoalPose; // We store estimated goal pose here

    pfms::commands::Ackerman cmd { // We create the command here with zero values for the conrols
        0, 
        0,
        0,
        0.0,
    };

    //! We will use a state machine to control the platform
    while(!goalReached){

        //! This reads odometry from the platform
        bool OK  =  pfmsConnectorPtr->read(odo);

        if(!OK){
            std::cout << "Unable to read odometry .. check simulation" << std::endl;
            return 0;    
        }

        OK = audi.computeSteering(odo, goal, steering, distance);

        if(!OK){
            std::cout << "Unable to compute steering .. goal not reachable" << std::endl;
            state = control::State::STOPPING;
        }

        switch(state)
        {
            case control::State::IDLE   : 
                if(distance>tolerance){
                    state = control::State::ACCELERATE;
                }    
                break;
            case control::State::ACCELERATE : 
                if(distance<5.0*tolerance){ // If we are close to the goal, we can coast
                    state = control::State::COAST;
                }
                else{
                    cmd.brake = 0.0;
                    cmd.steering = steering;
                    cmd.throttle = 0.1;
                }
                break;
            case control::State::COAST : 
                if(distance<2.0*tolerance){ // If we are close to goal by some margin, we can stop
                    state = control::State::STOPPING;
                }
                else{
                    cmd.brake=0;
                    cmd.steering = steering;
                    cmd.throttle = 0.05; // This is a fixed value, you can think if a proportion control here would be better?
                }
                break;
            case control::State::STOPPING :
                // If we are at the goal, we can stop, but does below stop the platform?
                // If you just applied the break until your within tolerance would this stop the car
                // What should we be checking here to determine if we have stopped? (Look at pfms::nav_msgs::Odometry to see what tells us if car is still moving )
                if (distance<tolerance){ 
                    goalReached=true;
                }
                else{
                    cmd.brake = 9000; // This is max break torque, you can think if a proportion control here would be better? Proportioonal to what?
                    cmd.throttle = 0.0;
                    cmd.steering = 0.0;
                }
            default:
                break;
        }
        
        // This sends the command to the platform
        pfmsConnectorPtr->send(cmd);
        cmd.seq++;
        //! This slows down the loop to 100Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        std::cout << state << " d:" << distance << " vx:" << odo.linear.x << std::endl;

    }






   return 0;
}
