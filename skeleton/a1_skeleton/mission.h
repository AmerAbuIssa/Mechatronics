#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "missioninterface.h"
#include "pfmsconnector.h"

class Mission: public MissionInterface
{
public:
  /**
  The Default constructor
  @sa ControllerInterface and @sa MissionInterface for more information
  */
  Mission(std::vector<ControllerInterface*> controllers);
  
  /**
   * @brief Accepts the container of goals.
   * @param goals
   */
  void setGoals(std::vector<pfms::geometry_msgs::Point> goals) override;
  
  /**
   * @brief Runs the mission
   * @return bool indicating mission complete (false if mission not possible OR aborted because it
   * can not be completed )
   */
  bool runMission() override;
  
  /**
   * @brief Set mission objective
   */
  void setMissionObjective(mission::Objective objective) override;
  
  /**
   * @brief Returns a vector of same size as number of controllers (platforms).
   * The values in the vector correspond to the total distance travelled by the corresponding platform
   * from the time of starting the program.
   *
   * @return std::vector<double> - each elemtn distance travelled for each vehicle [m]
   *
   */
  std::vector<double> getDistanceTravelled() override;
  
  /**
   * @brief Returns a vector of same size as number of controllers (platforms).
   * The values in the vector correspond to the time the corresponding platfore has been moving
   * from the time the program started. Moving means the platform was not stationary.
   *
   * @return std::vector<double> - each elemtn distance travelled for each vehicle [m]
   *
   */
  std::vector<double> getTimeMoving() override;
  
  /**
   * @brief Returns a vector of same size as number of goals. The values in the vector
   * correspond to the platform number that is completing the goal
   *
   * @return vector of unsigned int's corresponds to platform number completing the goal
   */
  std::vector<unsigned int> getPlatformGoalAssociation() override;

private:
  std::vector<ControllerInterface*> controllers_; //!< A private copy of ControllerInterfaces @sa ControllerInterface
  std::vector<pfms::geometry_msgs::Point> goals_; //!< A private copy of goals
  mission::Objective objective_; //!< The mission objective (TIME or DISTANCE)
  std::vector<unsigned int> platformGoalAssociation_; //!< Maps goals to platforms
  
  /**
   * @brief Helper function to assign goals to platforms based on objective
   * @return bool indicating if assignment was successful
   */
  bool assignGoalsToPlatforms();
};

#endif // MISSION_H
