#include "rviz_plugin.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(atwork_commander::RefboxUI, rviz::Panel);

namespace atwork_commander {

RefboxUI::RefboxUI(QWidget* parent)
    : rviz::Panel(parent)
    , mainVG(parent)
{
    mainVG.registerSubclass(&taskGenVG, "Task Generator");
    setLayout(mainVG.getLayout());
}

} // atwork_commander
