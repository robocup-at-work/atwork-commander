#include <atwork_commander_gen/TaskGenerator.h>
#include <atwork_commander_gen/GeneratorPluginInterface.h>

#include <pluginlib/class_loader.h>

#include <ros/ros.h>

#include <stdexcept>

using namespace std;

namespace atwork_commander {

TaskGenerator::TaskGenerator(const std::string& arenaConfig, const std::string& taskConfig, const std::string& pluginConfig){
  pluginlib::ClassLoader<TaskGenerator::GeneratorPlugin> loader("atwork_commander_gen", "atwork_commander::task_generator::GeneratorPluginInterface");
  string generatorName;
  if(!ros::param::get(pluginConfig, generatorName))
    throw runtime_error(string("TaskGenerator Plugin Configuration '")+pluginConfig+"' invalid");
  mImpl.reset(loader.createUnmanagedInstance(generatorName.c_str()));
  mImpl->onInit(arenaConfig, taskConfig);
}

TaskGenerator::Task TaskGenerator::operator()(const string& taskName)   { return mImpl?mImpl->generate(taskName):Task(); }

bool TaskGenerator::check(const Task& task) const { return mImpl?mImpl->check(task):false; }
ConfigParserInterface& TaskGenerator::config() { if(!mImpl) throw runtime_error("No Plugin loaded"); return mImpl->config(); }

}
