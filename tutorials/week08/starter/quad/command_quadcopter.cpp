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
        " <move_f_b>" << move_f_b << endl;


    pfms::PlatformType type = pfms::PlatformType::QUADCOPTER;        
    std::shared_ptr<PfmsConnector> pfmsConnectorPtr = std::make_shared<PfmsConnector>(type);
    pfms::nav_msgs::Odometry odo;

    //! @todo Let's read the odometry
    //! @todo Keep sending takeoff command until the quadcopter is in the air (check the z value, or that we have sent 10 or so takeoffs - with a 200ms pause
    //! @todo Then send the pfms::commands::Quadcopter command to the quadcopter using requested values

    return 0;
}
