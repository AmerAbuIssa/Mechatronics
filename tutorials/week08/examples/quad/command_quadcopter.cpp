#include "pfmsconnector.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <chrono>

using std::cout;
using std::endl;
using pfms::commands::Quadcopter;

int main(int argc, char *argv[]) {

    int repeats = 100;
    double turn_l_r = 0.0;
    double move_l_r = 0.0;
    double move_u_d = 0.0;
    double move_f_b = 0.1;

    if(argc !=6){
         cout << " Not arguments given on command line." << endl;
    }
    else{
        repeats = atoi(argv[1]);
        turn_l_r = atof(argv[2]);
        move_l_r = atof(argv[3]);
        move_u_d = atof(argv[4]);
        move_f_b = atof(argv[5]);
        cout << "Using arguments for: " << argv[0] << endl;
    }    

    cout << "<repeats>" << repeats << 
        " <turn_l_r>" << turn_l_r << 
        " <move_l_r>" << move_l_r << 
        " <move_u_d>" << move_u_d << 
        "<move_f_b>" << move_f_b << endl;


    pfms::PlatformType type = pfms::PlatformType::QUADCOPTER;        
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr = std::make_shared<PfmsConnector>(type);
    pfms::nav_msgs::Odometry odo;

    //Let's take off here, we send the status to the platform, check that platform has taken off
    bool OK =  pfmsConnectorPtr->read(odo);
    while (!OK){
        std::cout << "Waiting for platform to be ready" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        OK =  pfmsConnectorPtr->read(odo);
    }

    // Just in case we timeout after 50 takeoff commands
    unsigned int takeoffCount = 0;
    // We send the take off command to the platform and check z>0
    while ((odo.position.z < 0.1) && (takeoffCount < 50)){
        std::cout << "Waiting for platform to take off" << std::endl;
        pfms::PlatformStatus status = pfms::PlatformStatus::TAKEOFF;
        pfmsConnectorPtr->send(status);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // Let's move it up, just in case it's taken off already and has not enough upward momentum
        Quadcopter cmd {takeoffCount,0,0,0.05,0}; 
        // Sending the commands
        pfmsConnectorPtr->send(cmd);
        OK =  pfmsConnectorPtr->read(odo);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        takeoffCount++;
    }

    unsigned long i = takeoffCount;

    /* We loop sending same message for the number of times requested */
    for(i = 0; i < repeats; i ++) {
        // We take the arguments supplied on command line and place them in the uav command message
        Quadcopter cmd {i,turn_l_r,move_l_r,move_u_d,move_f_b}; 
        // Sending the commands
        pfmsConnectorPtr->send(cmd);
        // We wait for a short time, just to enable remaining of system to respond, recommended to wait
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        bool OK =  pfmsConnectorPtr->read(odo);
        // We check if the odometry returned indicates the system is running
        if(!OK){
            break;
        }
        std::cout << "i time x,y,yaw,vx,vy: " <<
            i << " " <<
            odo.time << " " <<
            odo.position.x << " " <<
            odo.position.y << " " <<
            odo.yaw << " " <<
            odo.linear.x << " " <<
            odo.linear.y << " " <<
            odo.linear.z << std::endl;
        std::this_thread::sleep_for (std::chrono::milliseconds(10));
    }

    // At the end of moving the uav it is recommended to at least stop, and possibly land
    // CONSIDER: 
    //      How would hover? What command do you need to send and do you need to check odometry
    //      Would you land before hovering? What if you needed to land on a target?

    return 0;
}
