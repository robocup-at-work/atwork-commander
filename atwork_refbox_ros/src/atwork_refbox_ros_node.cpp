#include <ros/ros.h>

#include <atwork_refbox_ros_task_generator/TaskGenerator.h>

#include <atwork_refbox_ros_msgs/RobotState.h>
#include <atwork_refbox_ros_msgs/Task.h>
#include <atwork_refbox_ros_msgs/LoadTask.h>
#include <atwork_refbox_ros_msgs/GenerateTask.h>
#include <atwork_refbox_ros_msgs/StartTask.h>

#include <stdexcept>
#include <iostream>

enum class State {
  IDLE,
  GENERATED,
  REGISTERED,
  READY,
  RUNNING
};

std::ostream& operator<<( std::ostream& os, State s ) {
  switch ( s ) {
    case ( State::IDLE )      : return os << "IDLE";
    case ( State::GENERATED ) : return os << "GENERATED";
    case ( State::REGISTERED ): return os << "REGISTERED";
    case ( State::READY )     : return os << "READY";
    case ( State::RUNNING )   : return os << "RUNNING";
    default                   : return os << "UNKNOWN";
  }
}

namespace atwork_refbox_ros {

class StateTracker {

    ros::NodeHandle m_nh;

    TaskDefinitions m_task_map;
    ArenaDescription m_arena;
    std::shared_ptr<TaskGenerator> m_task_gen;
    Task m_current_task;
    State m_state = State::IDLE;
    ros::Time m_end;

    ros::Publisher m_send_task_pub;
    ros::Subscriber m_robot_state_sub;
    ros::ServiceServer m_start_task_service;
    ros::ServiceServer m_generate_task_service;
    ros::ServiceServer m_load_task_service;

public:
    StateTracker(ros::NodeHandle nh)
        : m_nh(nh)
    {
        readTaskList();
        readArenaDefinition();

        m_robot_state_sub = m_nh.subscribe("internal/robot_state", 1, &StateTracker::receiveRobotStateClb, this);

        m_send_task_pub = m_nh.advertise<atwork_refbox_ros_msgs::Task>("internal/task", 1);

        m_task_gen = std::make_shared<TaskGenerator>(m_arena, m_task_map);
        m_start_task_service = m_nh.advertiseService("refbox/internal/start_task", &StateTracker::startTask, this);
        m_generate_task_service = m_nh.advertiseService("refbox/internal/generate_task", &StateTracker::generateTask, this);
        m_load_task_service = m_nh.advertiseService("refbox/internal/load_task", &StateTracker::loadTask, this);
    }

    ~StateTracker() {}

private:
    void stateUpdate( State state ) {
      switch ( state ) {
        case ( State::GENERATED ):
          if ( m_state == State::IDLE ) {
            ROS_DEBUG_STREAM("[REFBOX] Switchign state to GENERATED");
            m_state = State::GENERATED;
            return;
          }
        default: ROS_ERROR_STREAM("[REFBOX] State change not yet implemented: " << state);
      }
      ROS_DEBUG_STREAM("[REFBOX] Keeping state: " << m_state);
    }

    void readTaskList()
    {
        std::string task_param = ros::this_node::getNamespace() + "/Tasks";
        ROS_INFO_STREAM("[atwork_refbox] Try to read task definitions from '" << task_param << "'");

        XmlRpc::XmlRpcValue my_list;
        m_nh.getParam(task_param, my_list);

        if (my_list.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR("[atwork_refbox] Couldn't read Tasklist. Aspect 'XmlRpc::XmlRpcValue::TypeStruct' under '%s'.", task_param.c_str());
            ROS_ASSERT(false);
        }

        for (auto& task_p : my_list) {
            std::string name = static_cast<std::string>(task_p.first);

            if (task_p.second.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_WARN_STREAM("[atwork_refbox] " << task_param << "/" << name << " is not a task definition");
                continue;
            }

            TaskDefinition task;

            for (auto& value_p : task_p.second) {
                if (value_p.second.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                    task[value_p.first] = value_p.second;
                    continue;
                }
                if (value_p.second.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
                    task[value_p.first] = (value_p.second) ? 1 : 0;
                    continue;
                }
                ROS_WARN_STREAM("[atwork_refbox] " << task_param << "/" << name << " has no valid type. Allowed is Int and Bool. Current is '" << value_p.second.getType() << "'");
            }

            if (task.size() > 0) {
                m_task_map[name] = task;
                ROS_INFO_STREAM("[atwork_refbox] Read task '" << name << "' with " << task.size() << " values");
            }
        }

        ROS_ASSERT(m_task_map.size() > 0);

        ROS_INFO_STREAM("[atwork_refbox] Read " << m_task_map.size() << " tasks from parameter server");
    }

    void readArenaDefinition()
    {
        std::string ws_param = ros::this_node::getNamespace() + "/Arena/Workstations";
        ROS_INFO_STREAM("[atwork_refbox] Try to read workstations from '" << ws_param << "'");

        std::map<std::string, std::string> temp_ws;
        m_nh.getParam(ws_param, temp_ws);
        for (auto& ws : temp_ws) {
            m_arena.workstations[ws.first] = ws.second;
        }

        ROS_ASSERT(m_arena.workstations.size() > 1);

        ROS_INFO_STREAM("[atwork_refbox] Read " << m_arena.workstations.size() << " workstations from parameter server");


        std::string ppc_param = ros::this_node::getNamespace() + "/Arena/PP_cavities";
        ROS_INFO_STREAM("[atwork_refbox] Try to read PP cavities from '" << ppc_param << "'");

        m_nh.getParam(ppc_param, m_arena.cavities);

        ROS_INFO_STREAM("[atwork_refbox] Read " << m_arena.cavities.size() << " PP cavities from parameter server");
    }

    void receiveRobotStateClb(const atwork_refbox_ros_msgs::RobotState::ConstPtr& msg)
    {
    }

    bool startTask(atwork_refbox_ros_msgs::StartTask::Request& req, atwork_refbox_ros_msgs::StartTask::Response& res) {
      return false;
    }

    bool generateTask(atwork_refbox_ros_msgs::GenerateTask::Request& req, atwork_refbox_ros_msgs::GenerateTask::Response& res) {
      if ( m_state == State::RUNNING ) {
        ROS_ERROR_STREAM("[REFBOX] Got generation request with ongoing task! Ignoring!");
        return false;
      }

      try {
        res.task = m_task_gen->operator()(req.task_name);
        m_current_task = res.task;
        ROS_DEBUG_STREAM("[REFBOX] New Task generated:" << std::endl << res.task);
        stateUpdate( State::GENERATED );
      } catch( std::runtime_error& e) {
        ROS_ERROR_STREAM("[REFBOX] Error generating requested task " << req.task_name << ":" << std::endl << e.what());
        return false;
      }

      return true;
    }

    bool loadTask(atwork_refbox_ros_msgs::LoadTask::Request& req, atwork_refbox_ros_msgs::LoadTask::Response& res) {
      return false;
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "atwork_refbox_ros");
    ros::NodeHandle nh;

    atwork_refbox_ros::StateTracker st(nh);
    ROS_INFO("[atwork_refbox] initialized");

    ros::spin();

    return 0;
}
