#include "rviz_plugin.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(atwork_refbox_ros::RefboxUI, rviz::Panel);

namespace atwork_refbox_ros {

RefboxUI::RefboxUI(QWidget* parent)
    : rviz::Panel(parent)
    , mainVG(parent)
{
    mainVG.registerSubclass(&taskGenVG, "Task Generator");
    setLayout(mainVG.getLayout());
}

VisGroup::~VisGroup()
{
    delete self;
    delete layout;
    delete message;
    self = nullptr;
    layout = nullptr;
    message = nullptr;
}

MainVisGroup::~MainVisGroup()
{
    delete mainComboBox;
    delete mainStackedWidget;
    delete refreshButton;
    delete buttonHBoxLayout;
    // delete timer;
    for (auto e : subGroups) {
        delete e;
    }
    mainComboBox = nullptr;
    mainStackedWidget = nullptr;
    refreshButton = nullptr;
    buttonHBoxLayout = nullptr;
    // timer             = nullptr;
}

TaskGenVisGroup::~TaskGenVisGroup()
{
    delete taskListCombo;
    delete pptCavatiesLineEdit;
    taskListCombo = nullptr;
    pptCavatiesLineEdit = nullptr;
}

} // atwork_refbox_ros
