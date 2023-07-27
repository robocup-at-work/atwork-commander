/*
 * Copyright (c) 2020 <copyright holder> <email>
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "RefboxStateVisualizer.h"

#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

namespace atwork_commander {

using namespace atwork_commander_msgs;

std::string prefix = "[REFBOX_VIZ] ";
std::string logName = "refbox_viz";

RefboxStateVisualizer::RefboxStateVisualizer() {
  
  ROS_INFO_STREAM_NAMED(logName, prefix << "Refbox Visualization starts...");  
  ros::NodeHandle nh("~");
  
  std::string refboxName = "atwork_commander";

  if(!nh.getParam("refbox", refboxName))
    ROS_WARN_STREAM_NAMED(logName, prefix << "No refbox specified using default: " << refboxName);

  std::string topic = "/"+refboxName+"/internal/state";

  ROS_DEBUG_STREAM_NAMED(logName, prefix << "Starting subscriber of refbox state on " << topic);
  mStateSub = nh.subscribe(topic, 1, &RefboxStateVisualizer::handleState, this);
  
  topic = "/" + refboxName + "/viz/robot_poses";
  ROS_DEBUG_STREAM_NAMED(logName, prefix << "Registering publisher for robot poses on " << topic);
  mRobotPosePub = nh.advertise<visualization_msgs::MarkerArray>(topic, 1);
  
}

void RefboxStateVisualizer::handleState(const RefboxState::ConstPtr& msg) {
    ROS_INFO_STREAM_THROTTLE_NAMED(60, "refbox_viz", prefix << "Current Task: " << msg->task.type << " with ID " << msg->task.id << " created with config " << msg->task.config << " on git commit " << msg->task.commit);
    visualization_msgs::MarkerArray robotPoses;
  for (const RobotState& robot : msg->robots) {
    ROS_INFO_STREAM_THROTTLE_NAMED(60, "refbox_viz", prefix << "Registered Robot: "
                                   << robot.sender.team_name << "/" << robot.sender.robot_name);
    robotPoses.markers.emplace_back();
    auto& poseMarker = robotPoses.markers.back();
    

    tf2::Quaternion quat;
    quat.setRPY(0, 0, robot.pose.theta);
    quat.normalize();

    poseMarker.pose.position.x = robot.pose.x;
    poseMarker.pose.position.y = robot.pose.y;
    poseMarker.pose.orientation = tf2::toMsg(quat);
    poseMarker.scale.x = 1;
    poseMarker.scale.y = 1;
    poseMarker.scale.z = 1;
    poseMarker.color.r = 0;
    poseMarker.color.g = 0;
    poseMarker.color.b = 1;
    poseMarker.color.a = 1;
    poseMarker.type = visualization_msgs::Marker::ARROW;
    poseMarker.action = visualization_msgs::Marker::ADD;
    poseMarker.ns = robot.sender.team_name + "/" + robot.sender.robot_name;
    poseMarker.header = robot.sender.header;


    for(const geometry_msgs::TransformStamped& trans : robot.sender.transforms) {
      geometry_msgs::TransformStamped t = trans;
      if(t.header.frame_id != "world")
        t.header.frame_id = robot.sender.team_name + "/" + robot.sender.robot_name + "/" + t.header.frame_id;
      t.child_frame_id =  robot.sender.team_name + "/" + robot.sender.robot_name + "/" + t.child_frame_id;
      mTFBroadcaster.sendTransform(t);
    }
  }
  mRobotPosePub.publish(robotPoses);
}
}
