#include "rviz_plugin.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(atwork_refbox_ros::RefboxUI, rviz::Panel);

namespace atwork_refbox_ros {

RefboxUI::RefboxUI(QWidget* parent)
    : rviz::Panel(parent)
    , mainVG(parent)
{
    mainVG.registerSubclass(&worldVG, "World");
    setLayout(mainVG.getLayout());
}

} // atwork_refbox_ros
