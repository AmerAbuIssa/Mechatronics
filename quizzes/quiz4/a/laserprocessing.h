#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <pfms_types.h>
#include <pfmsconnector.h>
#include <math.h>

class LaserProcessing
{
public:
  /*! @brief Constructor that allocates internals
   *
   *  @param[in]    pfmsConnectorPtr  Pointer to the pfms connector
   */
  LaserProcessing(std::shared_ptr<PfmsConnector> pfmsConnectorPtr);


  /*! @brief Obstacles can be approximated by the centre of all the valid laser readings (range vector). 
   * A reading is valid if it is reported as being in between the minimum and maximum range. 
   * For details of using laser data refer to Week 8 quad example, where we obtained and examine laser data. I
   *
   * @return obstacles in the laser scan data in the coordinates of the scanner (which is same as robot)
   */
   std::vector<pfms::geometry_msgs::Point> getObstacles(void);

private:

  /*! @brief Aquire a new laserScan
   */
  void newScan();

  std::shared_ptr<PfmsConnector> pfmsConnectorPtr_;
  pfms::sensor_msgs::LaserScan laserScan_;
};

#endif // LASERPROCESSING_H
