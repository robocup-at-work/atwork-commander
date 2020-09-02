#include <atwork_commander_gen/TaskGenerator.h>
#include <atwork_commander_gen/PluginInterface.h>

#include <pluginlib/class_loader.h>

#include <ros/ros.h>

#include <stdexcept>

using namespace std;

namespace atwork_commander {

TaskGenerator::TaskGenerator(const std::string& arenaConfig, const std::string& taskConfig, const std::string& pluginConfig){
  pluginlib::ClassLoader<TaskGenerator::Interface> loader("atwork_commander_gen", "atwork_commander::task_generator::PluginInterface");
  string generatorName;
  if(!ros::param::get(pluginConfig, generatorName))
    throw runtime_error(string("TaskGenerator Plugin Configuration '")+pluginConfig+"'invalid");
  mImpl.reset(loader.createUnmanagedInstance(generatorName.c_str()));
}

TaskGenerator::Task TaskGenerator::operator()(const string& taskName)   { return mImpl?mImpl->generate(taskName):Task(); }

bool TaskGenerator::check(const Task& task) const { return mImpl?mImpl->check(task):false; }

}
