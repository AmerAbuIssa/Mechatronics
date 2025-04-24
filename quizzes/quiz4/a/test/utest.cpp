#include "gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "quadcopter.h" // The quadcopter
#include "pfms_types.h" //A1 types
#include "test_helper.h" // Helper header that assembled the message
#include "pfmshog.h" // Controlling the simulator
#include "laserprocessing.h" // Processing the laser scan data


using namespace pfms::nav_msgs;

TEST(QuadcopterTest, Constructor) {

    //We create the PfmHog object pointer and use it to set initial pose of Quadcopter for test
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::QUADCOPTER);
    Odometry initial_odo = populateOdo(3,-3,0,M_PI/4);
    pfmsHogPtr->teleport(initial_odo);
    
    //Create a quadcopter and push back to controllers
    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());
    double tolerance =0.2;

    
    EXPECT_FLOAT_EQ(controllers.at(0)->distanceToGoal(),0);//Should be zero, even if we have no goals
    EXPECT_FLOAT_EQ(controllers.at(0)->timeToGoal(),0); //Should be zero, as we have no goals
    EXPECT_FLOAT_EQ(controllers.at(0)->timeTravelled(),0); //Should be zero, we have not travelled
    EXPECT_FLOAT_EQ(controllers.at(0)->distanceTravelled(),0); //Should be zero, we hae not travelled
    EXPECT_EQ(controllers.at(0)->getPlatformType(),pfms::PlatformType::QUADCOPTER);//Quadcopter type
    EXPECT_EQ(controllers.at(0)->status(),pfms::PlatformStatus::IDLE);//Should be IDLE, we have not started
    EXPECT_TRUE(controllers.at(0)->setTolerance(0.2)); //Should be true, we can set this tolerance
    EXPECT_FALSE(controllers.at(0)->setTolerance(-1)); //Should be false, we can not set negative tolerance

    //If we call getOdometry we expect it to retrun the location we set as initial odometry
    pfms::nav_msgs::Odometry odo = controllers.at(0)->getOdometry();
    EXPECT_NEAR(odo.position.x,initial_odo.position.x,tolerance); 
    EXPECT_NEAR(odo.position.y,initial_odo.position.y,tolerance);
    EXPECT_NEAR(odo.position.z,initial_odo.position.z,tolerance);
}

TEST(QuadcopterTest, CheckObstacles) {

    //We create the PfmHog object pointer and use it to set initial pose of Quadcopter for test
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::QUADCOPTER);
    Odometry initial_odo = populateOdo(5,-42,0, -M_PI/4);
    pfmsHogPtr->teleport(initial_odo);

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());

    std::vector<pfms::geometry_msgs::Point> obstacles = controllers.front()->getObstacles();

    //Print the obstacles
    std::cout << "Obstacles: " << obstacles.size() <<  std::endl;
    for (const auto& obstacle : obstacles) {
        std::cout << "Obstacle at (" << obstacle.x << ", " << obstacle.y << ", " << obstacle.z << ")" << std::endl;
    }

    ASSERT_EQ(obstacles.size(),1); // This will be the size of the laser scan of obstacles
    ASSERT_NEAR(obstacles.at(0).x,5,0.2); // This will be the size of the laser scan of obstacles
    ASSERT_NEAR(obstacles.at(0).y,-49.5,0.2); // This will be the size of the laser scan of obstacles

}

TEST(QuadcopterTest, StartingLogic) {

    //We create the PfmHog object pointer and use it to set initial pose of Quadcopter for test
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::QUADCOPTER);
    Odometry initial_odo = populateOdo(5,5,0, -M_PI/2);
    pfmsHogPtr->teleport(initial_odo);

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());
    controllers.front()->setTolerance(0.7);

    std::vector<pfms::geometry_msgs::Point> goals;
    goals.push_back({ 0, 0, 1});
    
    //We set the goal for the PfmsHog (Which will be used to check if the goal is reached or not)
    pfmsHogPtr->setGoals(goals);

    // Before we set the goal the platform shoudl be IDLE
    EXPECT_EQ(controllers.at(0)->status(),pfms::PlatformStatus::IDLE);//Should be IDLE, we have not started

    // We send the goals to the controller
    bool reachable = controllers.front()->setGoals(goals);

    // The goal should be reachable
    EXPECT_TRUE(reachable);

    // The platform should still be IDLE as we have not started the controller via run()
    EXPECT_EQ(controllers.at(0)->status(),pfms::PlatformStatus::IDLE);

    // As we are employing threading, we want to check that the run call does not block
    auto start_time = std::chrono::system_clock::now();

    // The below should not block, a call to run() should return promptly.
    controllers.front()->run();

    // We can check that the run() call has returned by checking the time taken
    auto current_time = std::chrono::system_clock::now();
    auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time);

    // We should have taken less than 2 seconds to start the controller, this is a conservative estimate
    EXPECT_LT(time_taken.count(),2.0);

    // Now we should be in RUNNING state
    EXPECT_EQ(controllers.at(0)->status(),pfms::PlatformStatus::RUNNING);

    //Let's pause for two seconds to allow the controller to start
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Given we called run(), two seconds ago, we should have a goal to reach and have started moving
    EXPECT_GT(controllers.at(0)->distanceToGoal(),0); // Should be greater than 0
    EXPECT_GT(controllers.at(0)->timeToGoal(),0); // Should be greater than 0
    EXPECT_GT(controllers.at(0)->distanceTravelled(),0); // Should be greater than 0
    EXPECT_GT(controllers.at(0)->timeTravelled(),0); // Should be greater than 0
    
    // The paltform should be in motion
    //If we call getOdometry we expect it to retrun the location different to initial odometry
    pfms::nav_msgs::Odometry odo = controllers.at(0)->getOdometry();
    double tolerance =0.05;
    EXPECT_GT(std::fabs(odo.position.x-initial_odo.position.x),tolerance); 
    EXPECT_GT(std::fabs(odo.position.y-initial_odo.position.y),tolerance);
    EXPECT_GT(std::fabs(odo.position.z-initial_odo.position.z),tolerance);//While technically this shoudl be only positive, lets do a fabs  
}

TEST(QuadcopterTest, CheckObstaclesMoving) {

    //We create the PfmHog object pointer and use it to set initial pose of Quadcopter for test
    std::unique_ptr<PfmsHog> pfmsHogPtr = std::make_unique<PfmsHog>(pfms::PlatformType::QUADCOPTER);
    Odometry initial_odo = populateOdo(5,-42,0, -M_PI/4);
    pfmsHogPtr->teleport(initial_odo);

    std::vector<ControllerInterface*> controllers;
    controllers.push_back(new Quadcopter());
    controllers.front()->setTolerance(0.5);

    std::vector<pfms::geometry_msgs::Point> goals;
    goals.push_back({ 5, -47, 1.0});
    
    //We set the goal for the PfmsHog (Which will be used to check if the goal is reached or not)
    pfmsHogPtr->setGoals(goals);

    //We send the goals to the controller
    controllers.front()->setGoals(goals);

    // We will loop until that time and if the goal is not reached until then (or we have status)
    // indicating IDLE, we know it has been reached, we use a max time of 180s to reach it
    auto start_time = std::chrono::system_clock::now();
    double maxTime = 180.0;

    // The below should not block and we will be back and can check progress
    controllers.front()->run();
    bool OK =false; // This will indicate if mission is completed
    bool timeExceeded = false; // time exceeded


    while (!OK){

        auto current_time = std::chrono::system_clock::now();
        //std::chrono::seconds is integer for some reason, thus duration<double>
        auto time_taken = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time);

        if(time_taken.count()>(maxTime)){
            //We have now taken time too much time, and we terminate 
            timeExceeded=true;
            break;
        }

        pfms::PlatformStatus status = controllers.front()->status();

        if(status == pfms::PlatformStatus::IDLE){
            OK=true; // mission accomplished
        }

        //Let's slow down this loop to 200ms (5Hz)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    ASSERT_FALSE(timeExceeded); // time should not be exceeded
    //We now check that the goal has been reached using the PfmsHog
    std::vector<double> distances;
    bool reachedCheck = pfmsHogPtr->checkGoalsReached(distances);
    ASSERT_TRUE(reachedCheck);

    //We can also check the distance to the goal reported by PfmsHog
    ASSERT_GE(distances.size(),1);
    ASSERT_NEAR(distances.at(0),0,1.0);

    std::vector<pfms::geometry_msgs::Point> obstacles = controllers.front()->getObstacles();

    ASSERT_EQ(obstacles.size(),1); // This will be the size of the laser scan of obstacles
    ASSERT_NEAR(obstacles.at(0).x,5,0.2); // This will be the size of the laser scan of obstacles
    ASSERT_NEAR(obstacles.at(0).y,-49.5,0.2); // This will be the size of the laser scan of obstacles

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
