#pragma once

#include "ros/ros.h"

#include "atwork_commander_msgs/Task.h"
#include "atwork_commander_msgs/RobotState.h"

typedef std::function<void( atwork_commander_msgs::RobotState )> robot_state_fct_t;

namespace atwork_commander {
namespace com_plugin {

class Base {
  /** Callback executed if a new atwork_commander::RobotState was received **/
  robot_state_fct_t robot_state_fct;

public:

  virtual ~Base() {}

  /**Initalize the global variables.
   * \param roshandle a valid ROS NodeHandle
   * \param rsf callback to handle a newly received atwork_commander_msgs::RobotState
   **/
  void initialize( ros::NodeHandle roshandle, robot_state_fct_t rsf ) {
    onInit(roshandle);
    robot_state_fct = rsf;
  }

  virtual void sendTask( atwork_commander_msgs::Task task ) = 0;

protected:

  /**In this function the plugIn can do some initialisation
   * depending on a ROS NodeHandle.
   * @param roshandle a valid ROS NodeHandle
   **/
  virtual void onInit( ros::NodeHandle roshandle ) {}

  void sendRobotState( atwork_commander_msgs::RobotState robot_state ) {
    this->robot_state_fct( robot_state );
  }

};

}; //ns com_plugin
}; //ns atwork_commander
