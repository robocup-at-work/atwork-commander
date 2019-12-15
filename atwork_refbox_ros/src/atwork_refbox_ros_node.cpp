#include <atwork_refbox_task_generator/TaskGenerator.h>
#include <ros/ros.h>

namespace atwork_refbox_ros {

class StateTracker {
    ros::NodeHandle m_nh;

    TaskDefinitions m_task_map;

public:
    StateTracker(ros::NodeHandle nh)
        : m_nh(nh)
    {
        readTaskList();

        /** read arena definition **/
    }

    ~StateTracker() {}

private:
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
