#include "rviz_plugin.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(atwork_commander::RefboxUI, rviz::Panel);

namespace atwork_commander {

RefboxUI::RefboxUI(QWidget* parent)
    : rviz::Panel(parent)
    , mainVG(parent)
{
    //! init communication to core
    control_ptr = std::make_shared<atwork_commander::Control>();

    std::string refboxName = "atwork_commander";
    if (ros::param::get("~refbox", refboxName))
        control_ptr->refbox(refboxName);
    else
        ROS_WARN_STREAM_NAMED("control", "[REFBOX-CONTROL] No Refbox name specified using \"" << control_ptr->refbox() << "\"!");

    // control_ptr->stateUpdateCallback(&stateUpdate); //TODO

    //! qt
    mainVG.registerSubclass(&taskGenVG, "Task Generator");
    mainVG.registerSubclass(&robotGenVG, "Robot Control");
    mainVG.registerSubclass(&arenaGenVG, "Arena State");
    setLayout(mainVG.getLayout());

    mainVG.setControlPtr(control_ptr);
}

} // atwork_commander
