#pragma once

#include "ros/ros.h"

#include "atwork_refbox_ros_msgs/Task.h"
#include "atwork_refbox_ros_msgs/RobotState.h"

typedef std::function<void( atwork_refbox_ros_msgs::RobotState )> robot_state_fct_t;

namespace atwork_refbox_ros {
namespace com_plugin {

class ComInterface {

  robot_state_fct_t robot_state_fct;

protected:

  ComInterface() {}

public:

  virtual ~ComInterface() {}

  /**Initalize the global variables.
   * @param roshandle a valid ROS NodeHandle
   **/
  void initialize( ros::NodeHandle roshandle, robot_state_fct_t rsf ) {
    onInit(roshandle);
    rsf = robot_state_fct;
  }

  virtual void sendTask( atwork_refbox_ros_msgs::Task task ) = 0;

protected:

  /**In this function the plugIn can do some initialisation
   * depending on a ROS NodeHandle.
   * @param roshandle a valid ROS NodeHandle
   **/
  virtual void onInit( ros::NodeHandle roshandle ) {}

  void sendRobotState( atwork_refbox_ros_msgs::RobotState robot_state ) {
    this->robot_state_fct( robot_state );
  }

};

}; //ns com_plugin
}; //ns atwork_refbox
