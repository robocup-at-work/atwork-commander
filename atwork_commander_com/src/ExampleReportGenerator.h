#include "atwork_commander_msgs/RobotState.h"
#include "atwork_commander_msgs/Task.h"

#include "ros/ros.h"

#include <iostream>

using  atwork_commander_msgs::RobotState;
using  atwork_commander_msgs::Task;
using  atwork_commander_msgs::Workstation;
using  atwork_commander_msgs::Object;

class ExampleReportGenerator {
  protected:
    RobotState& mState;
    const Task& mTask;
    unsigned long mTaskID = 0;

  public:
    ExampleReportGenerator(RobotState& state, const Task& task)
      : mState(state), mTask(task) {
      mState.sender.transforms.emplace_back();
      auto& t = mState.sender.transforms.back();
      t.header.frame_id = "world";
      t.child_frame_id = "map";
      t.transform.rotation.w = 1.0;
    }

    void update() {
      //Update timestamps
      auto now = ros::Time::now();
      mState.sender.header.stamp = now;
      for(auto& t: mState.sender.transforms)
        t.header.stamp = now;

      //Check for arrival of new task
      if (mTask.id != mTaskID) {
        mTaskID = mTask.id;
        mState.current_arena_state = mTask.arena_start_state;
        ROS_INFO_STREAM_NAMED("example", "[REFBOX_EXAMPLE] start handling new task:" << std::endl << mTask );
        //Drop all objects
        mState.objects_on_robot[0]=Object();
        return;
      }

      //Deliver inventory of robot
      if(mState.objects_on_robot[0].object != Object::EMPTY) {
        
        ROS_DEBUG_STREAM_NAMED("example", "[REFBOX_EXAMPLE] got something in inventory: Delivering:" << std::endl << mState.objects_on_robot[0] );
        for(const auto& ws : mTask.arena_target_state)
          for(const auto& obj : ws.objects)
            if(obj.object == mState.objects_on_robot[0].object) {
              ROS_DEBUG_STREAM_NAMED("example", "[REFBOX_EXAMPLE] found goal object on workstation " << std::endl << ws.name );
              auto it = find_if(mState.current_arena_state.begin(), mState.current_arena_state.end(),
                                [&ws](const Workstation& reportWS){ return reportWS.name == ws.name;}
                        );
              if( it == mState.current_arena_state.end()) {
                ROS_DEBUG_STREAM_NAMED("example", "[REFBOX_EXAMPLE] workstation does not exist in report! Creating it");
                mState.current_arena_state.emplace_back();
                it = mState.current_arena_state.end()-1;
                it->name = ws.name;
              }
              ROS_DEBUG_STREAM_NAMED("example", "[REFBOX_EXAMPLE] Putting object " << obj << " on workstation " << *it);
              it->objects.push_back(obj);

              it->objects.back().pose.header.stamp = ros::Time::now();
              it->objects.back().pose.header.frame_id = "map";
              it->objects.back().pose.pose.orientation.w = 1.0;
              mState.objects_on_robot[0]=Object();
            }
        return;
      } 

      // Get next object
      ROS_DEBUG_STREAM_NAMED("example", "[REFBOX_EXAMPLE] inventory empty get next object!" );
      for(auto& ws : mState.current_arena_state) {
        for(const auto& obj: ws.objects) {
            auto it = find_if(mTask.arena_target_state.begin(), mTask.arena_target_state.end(),
                              [&ws](const Workstation& taskWS){ return taskWS.name == ws.name;}
                      );
            if(it != mTask.arena_target_state.end())
              if(count(it->objects.begin(), it->objects.end(), obj))
                continue;
            ROS_DEBUG_STREAM_NAMED("example", "[REFBOX_EXAMPLE] next object on " << ws.name << ":" << std::endl << ws.objects.back());
            Object& invObj = mState.objects_on_robot[0];
            invObj = ws.objects.back();
            invObj.pose.header.stamp = ros::Time::now();
            invObj.pose.header.frame_id = "map";
            invObj.pose.pose.orientation.w=1.0;
            ws.objects.pop_back();
            return;
        }
      } 
      ROS_DEBUG_STREAM_THROTTLE_NAMED(60, "example", "[REFBOX_EXAMPLE] task done!" );
    }


};
