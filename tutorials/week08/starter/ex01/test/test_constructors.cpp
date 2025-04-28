#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "ackerman.h"
#include "skidsteer.h"
#include "pfms_types.h"
#include <cmath>

//using namespace std;
using namespace pfms::nav_msgs;


//Test the constructor of the Ackerman class
TEST(ConstructorTest, Ackerman) {

    //Create a quadcopter and push back to controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());

    //! @todo What should the constructor set? check the values via the getters

}

//Test the constructor of the Ackerman class
TEST(ConstructorTest, SkidSteer) {

    //Create a quadcopter and push back to controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new SkidSteer());

    //! @todo What should the constructor set? check the values via the getters
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
