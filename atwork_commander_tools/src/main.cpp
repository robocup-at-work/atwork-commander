#include "ros/ros.h"
#include "atwork_commander_msgs/ObjectTask.h"
#include "atwork_commander_msgs/ObjectName.h"

#include <fstream>
#include <chrono>

const std::string FILE_OUTPUT_PATH = "/tmp/"; /**< path of the output file */

/**
 * @brief append a string on a file
 * @param[in] filename name of file to write to
 * @param[in] line data to write
 */
void write2File(std::string filename, std::string line) {
    std::ofstream file;
    file.open(FILE_OUTPUT_PATH+filename, std::ios::app);
    file << line;
    file.close();
}

/**
 * @brief subscriber callback which converts the message to a human readable format
 * and write a csv file
 * @param[in] ObjectTask message
 */
void objectTaskCallback(const atwork_commander_msgs::ObjectTask::ConstPtr& msg) {
    std::time_t result = std::time(nullptr); // get time for output file name
    std::string filename(std::asctime(std::localtime(&result))); 
    filename = "task_" + filename + ".csv";

    write2File(filename, "object, source, destination, placement target\n");

    int numberOfSubtasks = msg->subtasks.size();
    for(int i = 0; i < numberOfSubtasks; i++) {
        std::string object(atwork_commander_msgs::objectName(msg->subtasks[i].object.object));
        std::string placementTarget(atwork_commander_msgs::objectName(msg->subtasks[i].object.target));
        ROS_INFO_STREAM(    "[TASK_OUTPUT] object " + object
                            + " from workstation " + msg->subtasks[i].source 
                            + " to workstation " + msg->subtasks[i].destination
                            + (" in placement target " + placementTarget) );
        write2File(filename, object+","+msg->subtasks[i].source+","+msg->subtasks[i].destination+","+placementTarget+"\n");
    }

    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "task_output");

    ros::NodeHandle nh;

    ros::Subscriber subObjectTask = nh.subscribe("/atwork_commander/object_task" , 1000, objectTaskCallback);

    ROS_INFO_STREAM("[TASK_OUTPUT] startup :)");

    ros::spin();
}
