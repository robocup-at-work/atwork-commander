#include <ros/ros.h>

#include <atwork_refbox_ros_task_generator/TaskGenerator.h>

#include <atwork_refbox_ros_msgs/RobotState.h>
#include <atwork_refbox_ros_msgs/RefboxState.h>
#include <atwork_refbox_ros_msgs/Task.h>
#include <atwork_refbox_ros_msgs/LoadTask.h>
#include <atwork_refbox_ros_msgs/GenerateTask.h>
#include <atwork_refbox_ros_msgs/StartTask.h>

#include <stdexcept>
#include <iostream>
#include <numeric>
#include <unordered_map>

template<typename T>
std::ostream& operator<<( std::ostream& os, const std::vector<T>& v) {
  os << "[";
  for ( const T& t : v )
    os << t << " ";
  return os << "]";
}

namespace atwork_refbox_ros {

using State       = atwork_refbox_ros_msgs::RefboxState;
using RobotState  = atwork_refbox_ros_msgs::RobotState;
using RobotHeader = atwork_refbox_ros_msgs::RobotHeader;

class StateTracker {

    static std::string state2string( uint8_t state ) {
      switch( state ) {
        case( State::FAILURE )	  : return "FAILURE";
        case( State::IDLE )	      : return "IDLE";
        case( State::READY )	    : return "READY";
        case( State::PREPARATION) : return "PREPARATION";
        case( State::EXECUTION )  : return "EXECUTION";
        default                   : return "UNKNOWN";
      }
    }

    ros::NodeHandle m_nh;

    TaskDefinitions m_task_map;
    double m_robot_timeout = 1.0;
    ArenaDescription m_arena;
    std::shared_ptr<TaskGenerator> m_task_gen;
    State m_state;

    ros::Publisher m_send_task_pub;
    ros::Publisher m_state_pub;
    ros::Subscriber m_robot_state_sub;
    ros::ServiceServer m_start_task_service;
    ros::ServiceServer m_generate_task_service;
    ros::ServiceServer m_load_task_service;
    ros::Timer m_publish_timer;
    ros::Timer m_robot_timer;
    ros::Timer m_task_timer;

public:
    StateTracker(ros::NodeHandle nh)
        : m_nh(nh)
    {
        m_state.state = State::IDLE;

        readTaskList();
        readArenaDefinition();

        try {
          m_task_gen = std::make_shared<TaskGenerator>(m_arena, m_task_map);
        } catch( const std::exception& e) {
          ROS_ERROR_STREAM_NAMED("state_tracker", "Invalid Configuration specified to Task Generator:" << std::endl << e.what());
          abort();
        }

        if ( !ros::param::get("~robot_timeout", m_robot_timeout) )
          ROS_WARN_STREAM_NAMED("state_tracker", "[REFBOX] Robot Timeout unset (private parameter: \"robot_timeout\") using default value 1.0s");
        double publish_frequency = 1.0;
        if ( !ros::param::get("~publish_frequency", publish_frequency) )
          ROS_WARN_STREAM_NAMED("state_tracker", "[REFBOX] Publication Frequency unset (private parameter: \"publish_frequency\") using default value 1.0 Hz");
        m_robot_state_sub = m_nh.subscribe("internal/robot_state", 1, &StateTracker::receiveRobotStateClb, this);

        m_send_task_pub = m_nh.advertise<atwork_refbox_ros_msgs::Task>("internal/task", 1);
        m_state_pub = m_nh.advertise<atwork_refbox_ros_msgs::RefboxState>("internal/state", 1);

        m_start_task_service = m_nh.advertiseService("internal/start_task", &StateTracker::startTask, this);
        m_generate_task_service = m_nh.advertiseService("internal/generate_task", &StateTracker::generateTask, this);
        m_load_task_service = m_nh.advertiseService("internal/load_task", &StateTracker::loadTask, this);

        m_publish_timer = nh.createTimer( ros::Duration( publish_frequency ), &StateTracker::publishUpdate, this );
    }

    ~StateTracker() {}

private:

    void taskUpdate( const ros::TimerEvent& e) {
      stateUpdate();
    }

    void stateUpdate() {
      switch( m_state.state ) {
        case ( State::IDLE ):
          if ( !m_state.robots.empty() && m_task_gen->check(m_state.task) ) {
            ROS_DEBUG_STREAM_NAMED("state_tracker", "[REFBOX] Robots registered and task generated! I am READY :-)");
            m_state.state = State::READY;
          }
          break;
        case ( State::READY ):
          if ( m_state.robots.empty() || ! m_task_gen->check(m_state.task) ) {
            ROS_DEBUG_STREAM_NAMED("state_tracker", "[REFBOX] Lost all robots or generated task! Going back to IDLE!");
            m_state.state = State::IDLE;
            break;
          }
          if ( m_state.end > ros::Time::now() ) {
            ROS_DEBUG_STREAM_NAMED("state_tracker", "[REFBOX] Received start command going to PREPARATION ending at " << m_state.end );
            m_state.state = State::PREPARATION;
            m_task_timer = m_nh.createTimer( m_state.task.prep_time, &StateTracker::taskUpdate, this, true );
          }
          break;
        case ( State::PREPARATION ):
          if ( m_state.end > ros::Time::now() ) {
            ROS_DEBUG_STREAM_NAMED("state_tracker", "[REFBOX] Prep Time over going to EXECUTION ending at " << m_state.end );
            m_state.state = State::EXECUTION;
            m_state.end += m_state.task.exec_time;
            m_task_timer = m_nh.createTimer( m_state.task.exec_time, &StateTracker::taskUpdate, this, true );
          }
          break;
        case ( State::EXECUTION )  :
          if ( m_state.end < ros::Time::now() ) {
            ROS_DEBUG_STREAM_NAMED("state_tracker", "[REFBOX] Task Execution time over! Task finished! Going back to READY");
            m_state.state = State::READY;
          }
          break;
        case ( State::FAILURE )    : ROS_ERROR_STREAM_NAMED("state_tracker",
                                      "[REFBOX] In failed state, will not accept any commands! Please restart!");
                                     break;
        default                    : ROS_ERROR_STREAM_NAMED("state_tracker",
                                      "[REFBOX] Invalid state encountered " << m_state.state << "!");
      }
      ROS_DEBUG_STREAM_NAMED("state_tracker", "[REFBOX] Keeping state: " << state2string( m_state.state) );
    }

    bool notInState( std::vector<uint8_t> states ) const {
      return std::accumulate( states.begin(), states.end(), true,
              [this](bool b, uint8_t s){ return b && m_state.state != s; } );
    }

    bool inState( std::vector<uint8_t> states ) const {
      return std::accumulate( states.begin(), states.end(), false,
              [this](bool b, uint8_t s){ return b || m_state.state == s; } );
    }


    void publishUpdate ( const ros::TimerEvent& e) const {
      ROS_DEBUG_STREAM_NAMED("state_tracker", "[REFBOX] Publishing state update: " << std::endl <<
                                              "\tState: " << state2string( m_state.state ) << std::endl <<
                                              "\tEnd of Task: " << m_state.end << std::
                                              endl << "\tTask: " << m_state.task );
      m_state_pub.publish( m_state );
      if ( inState( { State::EXECUTION } ) )
        m_send_task_pub.publish( m_state.task );
    }

    void readTaskList()
    {
        std::string task_param = ros::this_node::getNamespace() + "/Tasks";
        ROS_INFO_STREAM_NAMED("state_tracker", "[REFBOX] Try to read task definitions from '" << task_param << "'");

        XmlRpc::XmlRpcValue my_list;
        m_nh.getParam(task_param, my_list);

        if (my_list.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR_NAMED("state_tracker", "[REFBOX] Couldn't read Tasklist. Aspect 'XmlRpc::XmlRpcValue::TypeStruct' under '%s'.", task_param.c_str());
            ROS_ASSERT(false);
        }

        for (auto& task_p : my_list) {
            std::string name = static_cast<std::string>(task_p.first);

            if (task_p.second.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_WARN_STREAM_NAMED("state_tracker", "[REFBOX] " << task_param << "/" << name << " is not a task definition");
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
                ROS_WARN_STREAM_NAMED("state_tracker", "[REFBOX] " << task_param << "/" << name << " has no valid type. Allowed is Int and Bool. Current is '" << value_p.second.getType() << "'");
            }

            if (task.size() > 0) {
                m_task_map[name] = task;
                ROS_INFO_STREAM_NAMED("state_tracker", "[REFBOX] Read task '" << name << "' with " << task.size() << " values");
            }
        }

        ROS_ASSERT(m_task_map.size() > 0);

        ROS_INFO_STREAM_NAMED("state_tracker", "[REFBOX] Read " << m_task_map.size() << " tasks from parameter server");
    }

    void readArenaDefinition()
    {
        std::string ws_param = ros::this_node::getNamespace() + "/Arena/Workstations";
        ROS_INFO_STREAM_NAMED("state_tracker", "[REFBOX] ry to read workstations from '" << ws_param << "'");

        std::map<std::string, std::string> temp_ws;
        m_nh.getParam(ws_param, temp_ws);
        for (auto& ws : temp_ws) {
            m_arena.workstations[ws.first] = ws.second;
        }

        ROS_ASSERT(m_arena.workstations.size() > 1);

        ROS_INFO_STREAM_NAMED("state_tracker", "[REFBOX] Read " << m_arena.workstations.size() << " workstations from parameter server");


        std::string ppc_param = ros::this_node::getNamespace() + "/Arena/PP_cavities";
        ROS_INFO_STREAM_NAMED("state_tracker", "[REFBOX] ry to read PP cavities from '" << ppc_param << "'");

        m_nh.getParam(ppc_param, m_arena.cavities);

        ROS_INFO_STREAM_NAMED("state_tracker", "[REFBOX] ead " << m_arena.cavities.size() << " PP cavities from parameter server");
    }

    void receiveRobotStateClb(const atwork_refbox_ros_msgs::RobotState::ConstPtr& msg)
    {
      //TODO Find robot and update
      auto it = find_if(m_state.robots.begin(), m_state.robots.end(), [msg](const RobotHeader& r){ return r.team_name == msg->sender.team_name && r.robot_name == msg->sender.robot_name; });
      if ( it == m_state.robots.end() )
        m_state.robots.emplace_back(msg->sender);
      else
        *it = msg->sender;
      stateUpdate();
    }

    bool startTask(atwork_refbox_ros_msgs::StartTask::Request& req, atwork_refbox_ros_msgs::StartTask::Response& res) {
      if ( inState( { State::PREPARATION, State::EXECUTION } ) ) {
        ROS_ERROR_STREAM_NAMED("state_tracker", "[REFBOX] Got start request with ongoing task! Ignoring!");
        return false;
      }

      try {
        //TODO check suplied robots
        m_state.task.execute_on = req.robots;
        m_state.end = ros::Time::now() + m_state.task.prep_time;
        ROS_DEBUG_STREAM_NAMED("state_tracker", "[REFBOX] Start Task on robots: " << std::endl << req.robots);
        stateUpdate();
      } catch( std::runtime_error& e) {
        ROS_ERROR_STREAM_NAMED("state_tracker", "[REFBOX] Supplied robots invalid:" << std::endl << e.what());
        return false;
      }

      return true;
    }

    bool generateTask(atwork_refbox_ros_msgs::GenerateTask::Request& req, atwork_refbox_ros_msgs::GenerateTask::Response& res) {
      if ( inState( { State::PREPARATION, State::EXECUTION } ) ) {
        ROS_ERROR_STREAM_NAMED("state_tracker", "[REFBOX] Got generation request with ongoing task! Ignoring!");
        return false;
      }

      try {
        res.task = m_task_gen->operator()(req.task_name);
        m_state.task = res.task;
        ROS_DEBUG_STREAM_NAMED("state_tracker", "[REFBOX] New Task generated:" << std::endl << res.task);
        stateUpdate();
      } catch( std::runtime_error& e) {
        ROS_ERROR_STREAM_NAMED("state_tracker", "[REFBOX] Error generating requested task " << req.task_name << ":" << std::endl << e.what());
        return false;
      }

      return true;
    }

    bool loadTask(atwork_refbox_ros_msgs::LoadTask::Request& req, atwork_refbox_ros_msgs::LoadTask::Response& res) {
      if ( inState( { State::PREPARATION, State::EXECUTION } ) ) {
        ROS_ERROR_STREAM_NAMED("state_tracker", "[REFBOX] Got load request with ongoing task! Ignoring!");
        return false;
      }

      try {
        m_task_gen->check(req.task);
        m_state.task = req.task;
        ROS_DEBUG_STREAM_NAMED("state_tracker", "[REFBOX] Task loaded: " << std::endl << req.task);
        stateUpdate();
      } catch( std::runtime_error& e) {
        ROS_ERROR_STREAM_NAMED("state_tracker", "[REFBOX] Supplied task invalid:" << std::endl << e.what());
        return false;
      }

      return true;
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "atwork_refbox_ros");
    ros::NodeHandle nh;

    atwork_refbox_ros::StateTracker st(nh);
    ROS_INFO_NAMED("state_tracker", "[REFBOX] initialized");

    ros::spin();

    return 0;
}
