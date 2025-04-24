#include "laserprocessing.h"
#include <algorithm>
#include <numeric>

using namespace std;

LaserProcessing::LaserProcessing(std::shared_ptr<PfmsConnector> pfmsConnectorPtr):
    pfmsConnectorPtr_(pfmsConnectorPtr)
{
   
}

//! @todo TASK2 - Implement the getObstacles function
std::vector<pfms::geometry_msgs::Point> LaserProcessing::getObstacles(void)
{

    newScan();//This will populate the laser scan data
    std::vector<pfms::geometry_msgs::Point> obstacles;

    return obstacles;
}

void LaserProcessing::newScan(){
    // We need to read the laser scan data
    pfmsConnectorPtr_->read(laserScan_);
}

