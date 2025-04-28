#include "pfmsconnector.h"
#include <vector>
#include <iostream>
#include <thread>

int main(int argc, char *argv[]) {

    if(argc !=2){
        std::cout << " Not arguments given on command line." << std::endl;
        std::cout << " usage: " << argv[0] << " <repeats>" << std::endl;
        return 0;
    }

    //! Created a pointer to a Pipe (This is consumer and
    pfms::PlatformType type = pfms::PlatformType::QUADCOPTER;   
    std::unique_ptr<PfmsConnector> pfmsConnectorPtr = std::make_unique<PfmsConnector>(type);
    std::this_thread::sleep_for (std::chrono::seconds(1)); 

     for (unsigned int i=0;i<atoi(argv[1]);i++){
         pfms::sensor_msgs::LaserScan laserScan;
         try{

            //! @todo Let's read the laser scan
            //! @todo Use the laserScan message elements to compute range/bearing to any obstacles 
            //! @todo Find the closest obstacles and report the range/bearing to the platform

            std::this_thread::sleep_for (std::chrono::seconds(10));
        }
        catch(const std::exception& e){
            std::cerr << e.what() << '\n';
        }
     }

    return 0;
}
