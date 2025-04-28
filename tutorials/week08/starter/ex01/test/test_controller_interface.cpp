#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

//Student defined libraries
#include "ackerman.h"
//#include "skidsteer.h"
#include "pfms_types.h"
#include <cmath>
#include "a1_test_helper.h"
#include "pfmshog.h"


using namespace std;
using namespace pfms::nav_msgs;



///////////////////////////////////////////////////////////
// Unit Tests Start HERE
////////////////////////////////////////////////////////

TEST(ControllerInterface, Ackerman) {

    // This teleport Ackerman to a known location which is the start point
    pfms::PlatformType platform = pfms::PlatformType::ACKERMAN;
    //! Created a pointer to PfmsHog 
    std::shared_ptr<PfmsHog> pfmsHogPtr = std::make_shared<PfmsHog>(platform,true);
   {
       Odometry odo = populateOdo(0,5,0);
       pfmsHogPtr->teleport(odo);
   }

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Ackerman());
    //controllers.push_back(new Quadcopter());

    //Goal at x=10,y=0;
    pfms::geometry_msgs::Point pt{10,0};
    {
        //! @todo How do we check the controller interface functions, what is the output and how do we check it?

    }
    ASSERT_EQ(controllers.size(),1);

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

