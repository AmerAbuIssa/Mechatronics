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
        unsigned int i=0;
        bool reachable = controllers.at(i)->setGoal(pt);
        double dist = controllers.at(i)->distanceToGoal();
        double t = controllers.at(i)->timeToGoal();
        ASSERT_TRUE(reachable);

        pfms::PlatformType type = controllers.at(i)->getPlatformType();
        ASSERT_EQ(type,pfms::PlatformType::ACKERMAN);

        double distanceTravelled = controllers.at(i)->distanceTravelled();
        double timeInMotion = controllers.at(i)->timeInMotion();
        ASSERT_FLOAT_EQ(distanceTravelled,0.0f);
        ASSERT_FLOAT_EQ(timeInMotion,0.0f);

        pfms::nav_msgs::Odometry odo = controllers.at(i)->getOdometry();
        ASSERT_NEAR(odo.position.x,0.0,0.1);
        ASSERT_NEAR(odo.position.y,5.0,0.1);

        bool OK = controllers.at(i)->setTolerance(0.5);
        ASSERT_TRUE(OK);
        // Now if we set the tolerance to a negative value, it should return false
        OK = controllers.at(i)->setTolerance(-0.5);
        ASSERT_FALSE(OK);

    }
    ASSERT_EQ(controllers.size(),1);

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

