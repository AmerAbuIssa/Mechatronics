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

    //! Created a pointer to connector
    std::unique_ptr<PfmsConnector> pfmsConnectorPtr = std::make_unique<PfmsConnector>(pfms::PlatformType::QUADCOPTER);
    std::this_thread::sleep_for (std::chrono::seconds(5)); 

    bool firstMsg = false;
    for (unsigned int i=0;i<atoi(argv[1]);i++){
        pfms::sensor_msgs::Sonar sonar;
        try{
            //! @todo Let's read the sonar
            //! @todo Use the sonar message elements and show the valid reading on the screen
            std::this_thread::sleep_for (std::chrono::milliseconds(100));
        }
        catch (const std::runtime_error& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            return EXIT_FAILURE;
        }
    }
    return 0;
}
