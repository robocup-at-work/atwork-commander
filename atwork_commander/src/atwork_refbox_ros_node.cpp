#include <ros/ros.h>

#include <atwork_commander_gen/TaskGenerator.h>

#include <atwork_commander_msgs/RobotState.h>
#include <atwork_commander_msgs/RefboxState.h>
#include <atwork_commander_msgs/Task.h>
#include <atwork_commander_msgs/LoadTask.h>
#include <atwork_commander_msgs/GenerateTask.h>
#include <atwork_commander_msgs/StartTask.h>
#include <atwork_commander_msgs/StateUpdate.h>

#include <stdexcept>
#include <iostream>
#include <numeric>
#include <unordered_map>
#include <vector>
#include <thread>
#include <chrono>

template<typename T>
std::ostream& operator<<( std::ostream& os, const std::vector<T>& v) {
  os << "[";
  for ( const T& t : v )
    os << t << " ";
  return os << "]";
}

namespace atwork_commander {

using State       = atwork_commander_msgs::RefboxState;
using RobotState  = atwork_commander_msgs::RobotState;
using RobotHeader = atwork_commander_msgs::RobotHeader;

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
    bool m_debug = false;

    ros::Publisher m_send_task_pub;
    ros::Publisher m_state_pub;
    ros::Subscriber m_robot_state_sub;
    ros::ServiceServer m_start_task_service;
    ros::ServiceServer m_generate_task_service;
    ros::ServiceServer m_load_task_service;
    ros::ServiceServer m_state_update_service;
    ros::Timer m_publish_timer;
    ros::Timer m_robot_timer;
    ros::Timer m_task_timer;

public:
    StateTracker(ros::NodeHandle nh)
        : m_nh(nh)
    {
        ros::param::get("~debug", m_debug);
        if( m_debug && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
          ros::console::notifyLoggerLevelsChanged();
          std::this_thread::sleep_for(std::chrono::seconds(1)); // Necessary to allow rosconsole to react to logger level change
        }

        m_state.state = State::IDLE;

        readTaskList();
        readArenaDefinition();
        // TODO move to task generator
        for ( auto& item : m_task_map ) {
          auto& objects = item.second.objects;
          for ( const auto& obj : m_arena.objects ) {
            auto it = objects.find(obj.first);
            if (it == objects.end())
              objects.insert(obj);
          }
        }

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

        m_send_task_pub = m_nh.advertise<atwork_commander_msgs::Task>("internal/task", 1);
        m_state_pub = m_nh.advertise<atwork_commander_msgs::RefboxState>("internal/state", 1);

        m_start_task_service = m_nh.advertiseService("internal/start_task", &StateTracker::startTask, this);
        m_generate_task_service = m_nh.advertiseService("internal/generate_task", &StateTracker::generateTask, this);
        m_load_task_service = m_nh.advertiseService("internal/load_task", &StateTracker::loadTask, this);
        m_state_update_service = m_nh.advertiseService("internal/state_update", &StateTracker::externalStateUpdate, this);

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

   bool readStrings(StringList& data, XmlRpc::XmlRpcValue& val) const {
      if ( val.getType() != XmlRpc::XmlRpcValue::TypeArray ) {
        ROS_WARN_STREAM_NAMED("state_tracker", "[REFBOX] Invalid Parameter type encountered! Expected Array!");
        return false;
      }

      std::vector<std::string> strings( val.size() );
      auto it = strings.begin();

      for (int i=0;i<val.size();i++)
        switch(val[i].getType()) {
          case( XmlRpc::XmlRpcValue::TypeString ) : *it++ =                 static_cast<std::string>( val[i] )  ; break;
          case( XmlRpc::XmlRpcValue::TypeInt )    : *it++ = std::to_string( static_cast<int>        ( val[i] ) ); break;
          case( XmlRpc::XmlRpcValue::TypeDouble ) : *it++ = std::to_string( static_cast<double>     ( val[i] ) ); break;
          case( XmlRpc::XmlRpcValue::TypeBoolean ): *it++ = std::to_string( static_cast<bool>       ( val[i] ) ); break;
          default:
            ROS_WARN_STREAM_NAMED("state_tracker", "[REFBOX] Invalid Parameter type encountered for parameter in array!" <<
                                                           " Allowed are String, Int, Bool, Double" );
            return false;
        };

      return true;
   }

   bool readStruct(ParameterType& params, XmlRpc::XmlRpcValue& val) const {
      if ( val.getType() != XmlRpc::XmlRpcValue::TypeStruct ) {
        ROS_WARN_STREAM_NAMED("state_tracked", "[REFBOX] Invalid Parameter type encountered for parameter!" <<
                                                       " Expected Struct!");
        return false;
      }
      bool result = true;
      for (auto& entry: val) {
        switch(entry.second.getType()) {
          case( XmlRpc::XmlRpcValue::TypeInt )    : params[entry.first]=static_cast<int>( entry.second ); break;
          case( XmlRpc::XmlRpcValue::TypeBoolean ): params[entry.first]=static_cast<bool>( entry.second ); break;
          default:
            ROS_WARN_STREAM_NAMED("state_tracker", "[REFBOX] Invalid Parameter type encountered for struct parameter!" <<
                                                           " Allowed are Int and Bool");
            result = false;
        };
      }
      return result;
   }

   TaskDefinition readTask(const std::string& name, XmlRpc::XmlRpcValue& def ) const {

      TaskDefinition task;

      for (auto& entry : def) {
        if ( entry.first == "normal_table_types" ) {
          if ( !readStrings(task.normalTableTypes, entry.second) )
            ROS_WARN_STREAM_NAMED("state_tracker", "[REFBOX] Error reading table types of normale tables for task "
                                                   << name << "! Keeping default: " << task.normalTableTypes << "!");
          continue;
        }
        if ( entry.first == "tt_types" ) {
          if ( !readStrings(task.ttTypes, entry.second) )
            ROS_WARN_STREAM_NAMED("state_tracker", "[REFBOX] Error reading table types of turntables for task "
                                                   << name << "! Keeping default: " << task.ttTypes << "!");
          continue;
        }
        if ( entry.first == "pp_types" ) {
          if ( !readStrings(task.ppTypes, entry.second) )
            ROS_WARN_STREAM_NAMED("state_tracker", "[REFBOX] Error reading table types of precision placement for task " <<
                                                   name <<  "! Keeping default: " << task.ppTypes << "!");
          continue;
        }
        if ( entry.first == "sh_types" ) {
          if ( !readStrings(task.shTypes, entry.second) )
            ROS_WARN_STREAM_NAMED("state_tracker", "[REFBOX] Error reading table types of shelfs for task " << name <<
                                                   "! Keeping default: " << task.shTypes << "!");
          continue;
        }
        if ( entry.first == "cavities" ) {
          if ( !readStrings(task.cavities, entry.second) )
            ROS_WARN_STREAM_NAMED("state_tracker", "[REFBOX] Error reading available cavities for task " << name <<
                                                   "! Keeping default: " << task.cavities << "!");
          continue;
        }
        if ( entry.first == "object_types" ) {
          if ( !readStruct(task.objects, entry.second) ) {
            std::ostringstream os;
            for ( const auto& v: task.objects )
              os << "\t" << v.first << ": " << v.second << std::endl;
            ROS_WARN_STREAM_NAMED("state_tracker", "[REFBOX] Error reading available objects for task " << name <<
                                                   "! State after partial update: " << std::endl << os.str());

          }
          continue;
        }

        auto it = task.parameters.find(entry.first);
        if ( it != task.parameters.end()) {
          switch(entry.second.getType()) {
            case( XmlRpc::XmlRpcValue::TypeInt )    : it->second = static_cast<int>( entry.second ); break;
            case( XmlRpc::XmlRpcValue::TypeBoolean ): it->second = static_cast<bool>( entry.second ); break;
            default:
              ROS_ERROR_STREAM_NAMED("state_tracker", "[REFBOX] Invalid Parameter type encountered for parameter " <<
                                                      name << "/" << entry.first << "! Only int and bool is allowed!");
          };
        } else {
          ROS_WARN_STREAM_NAMED("state_tracker", "[REFBOX] Unknown parameter " << entry.first << " in definition of task " << name << "! Ignoring!");
        }
      }

      return task;
   }

    void readTaskList()
    {
        std::string task_param = ros::this_node::getNamespace() + "/tasks";
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

            m_task_map[name] = readTask(name, task_p.second);
        }

        ROS_ASSERT(m_task_map.size() > 0);
        ROS_INFO_STREAM_NAMED("state_tracker", "[REFBOX] Read " << m_task_map.size() << " tasks from paramet er server");
        ROS_DEBUG_STREAM_NAMED("state_tracker", "[REFBOX] Defined Tasks: " << m_task_map);
    }

    void readArenaDefinition()
    {
        std::string ws_param = ros::this_node::getNamespace() + "/arena/workstations";
        ROS_INFO_STREAM_NAMED("state_tracker", "[REFBOX] ry to read workstations from '" << ws_param << "'") ;

        std::string wp_param = ros::this_node::getNamespace() + "/arena/waypoints";
        ROS_INFO_STREAM_NAMED("state_tracker", "[REFBOX] ry to read waypoints from '" << wp_param << "'") ;

        std::string obj_param = ros::this_node::getNamespace() + "/arena/objects";
        ROS_INFO_STREAM_NAMED("state_tracker", "[REFBOX] ry to read available objects from '" << obj_param << "'") ;

        std::map<std::string, std::string> temp_ws;
        m_nh.getParam(ws_param, temp_ws);
        for (auto& ws : temp_ws) {
            m_arena.workstations[ws.first] = ws.second;
        }

        std::map<std::string, std::string> temp_wp;
        m_nh.getParam(wp_param, temp_wp);
        for (auto& wp : temp_wp) {
            m_arena.waypoints[wp.first] = wp.second;
        }

        std::map<std::string, int> temp_obj;
        m_nh.getParam(obj_param, temp_obj);
        for (auto& obj : temp_obj) {
            m_arena.objects[obj.first] = obj.second;
        }

        ROS_ASSERT(m_arena.workstations.size() > 1);

        ROS_INFO_STREAM_NAMED("state_tracker", "[REFBOX] Read " << m_arena.workstations.size() << " workstations from parameter server");


        std::string ppc_param = ros::this_node::getNamespace() + "/arena/cavities";
        ROS_INFO_STREAM_NAMED("state_tracker", "[REFBOX] try to read PP cavities from '" << ppc_param << "'");

        m_nh.getParam(ppc_param, m_arena.cavities);

        ROS_INFO_STREAM_NAMED("state_tracker", "[REFBOX] read " << m_arena.cavities.size() << " PP cavities from parameter server");
        ROS_DEBUG_STREAM_NAMED("state_tracker", "[REFBOX] Read Arena Definition:" << std::endl << m_arena);
    }

    void receiveRobotStateClb(const atwork_commander_msgs::RobotState::ConstPtr& msg)
    {
      //TODO Find robot and update
      auto it = find_if(m_state.robots.begin(), m_state.robots.end(), [msg](const RobotState& r){ return r.sender.team_name == msg->sender.team_name && r.sender.robot_name == msg->sender.robot_name; });
      if ( it == m_state.robots.end() )
        m_state.robots.emplace_back(*msg);
      else
        *it = *msg;
      stateUpdate();
    }

    bool startTask(atwork_commander_msgs::StartTask::Request& req, atwork_commander_msgs::StartTask::Response& res) {
      if ( notInState( { State::READY } ) ) {
        std::ostringstream os;
        if ( m_state.state == State::IDLE )
          os << "Got start request while refbox not ready! Ignoring!";
        else
          os << "Got start request with ongoing task! Ignoring!";
        ROS_ERROR_STREAM_NAMED("external", "[REFBOX] " << os.str());
        res.error = os.str();
        return true;
      }

      try {
        //TODO check suplied robots
        m_state.task.execute_on = req.robots;
        m_state.end = ros::Time::now() + m_state.task.prep_time;
        ROS_DEBUG_STREAM_NAMED("external", "[REFBOX] Start Task on robots: " << std::endl << req.robots);
        stateUpdate();
      } catch( const std::exception& e) {
        std::ostringstream os;
        os << "Supplied robots invalid:" << std::endl << e.what();
        ROS_ERROR_STREAM_NAMED("external", "[REFBOX] " << os.str());
        res.error = os.str();
        return true;
      }

      return true;
    }

    bool generateTask(atwork_commander_msgs::GenerateTask::Request& req, atwork_commander_msgs::GenerateTask::Response& res) {
      if ( inState( { State::PREPARATION, State::EXECUTION } ) ) {
        std::ostringstream os;
        os << "Got generation request with ongoing task! Ignoring!";
        ROS_ERROR_STREAM_NAMED("external", "[REFBOX] " << os.str());
        res.error = os.str();
        return true;
      }

      try {
        res.task = m_task_gen->operator()(req.task_name);
        m_state.task = res.task;
        ROS_DEBUG_STREAM_NAMED("external", "[REFBOX] New Task generated:" << std::endl << res.task);
        stateUpdate();
      } catch( const std::exception& e) {
        std::ostringstream os;
        os << "Error generating requested task " << req.task_name << ":" << std::endl << e.what();
        ROS_ERROR_STREAM_NAMED("external", "[REFBOX] " << os.str());
        res.error = os.str();
        return true;
      }

      return true;
    }

    bool loadTask(atwork_commander_msgs::LoadTask::Request& req, atwork_commander_msgs::LoadTask::Response& res) {
      if ( inState( { State::PREPARATION, State::EXECUTION } ) ) {
        ROS_ERROR_STREAM_NAMED("external", "[REFBOX] Got load request with ongoing task! Ignoring!");
        return false;
      }

      try {
        m_task_gen->check(req.task);
        m_state.task = req.task;
        ROS_DEBUG_STREAM_NAMED("external", "[REFBOX] Task loaded: " << std::endl << req.task);
        stateUpdate();
      } catch( const std::exception& e) {
        std::ostringstream os;
        os << "Supplied task invalid:" << std::endl << e.what();
        ROS_ERROR_STREAM_NAMED("external", "[REFBOX] " << os.str());
        res.error = os.str();
        return true;
      }

      return true;
    }
    bool externalStateUpdate( atwork_commander_msgs::StateUpdate::Request& req, atwork_commander_msgs::StateUpdate::Response& res) {
      if( req.state == m_state.state ) {
        return true;
      }
      if( ( req.state == State::EXECUTION && m_state.state == State::PREPARATION ) ||
          ( req.state == State::READY && m_state.state == State::PREPARATION ) ||
          ( req.state == State::READY && m_state.state == State::EXECUTION) ) {
        ROS_INFO_STREAM_NAMED("external", "[REFBOX] ending state manually!");
        m_state.end = ros::Time::now();
        stateUpdate();
        return true;
      }
      std::ostringstream os;
      os << "[REFBOX] Invalid state update received: " << ((size_t)m_state.state) << " -> " << ((size_t)req.state);
      res.error = os.str();
      ROS_ERROR_STREAM_NAMED("external", os.str());
      return true;
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "atwork_commander");
    ros::NodeHandle nh;

    atwork_commander::StateTracker st(nh);
    ROS_INFO_NAMED("state_tracker", "[REFBOX] initialized");

    ros::spin();

    return 0;
}
