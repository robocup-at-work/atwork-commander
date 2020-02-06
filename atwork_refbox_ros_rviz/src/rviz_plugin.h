
#pragma once

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QStackedWidget>
#include <QString>
#include <QTimer>
// add your QWidgets includes here

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>
#endif

namespace atwork_refbox_ros {

/*!
   * \brief Abstract Base Class VisGroup
   *
   * This baseclas specifies the interface for subclasses that should work within
   * the WorldmodelUI and provides a number of conviences (fucntions and members)
   */
class VisGroup : public QWidget {
    Q_OBJECT

protected:
    // self - the widget which will be parent of all in a VisGroup contained widgets
    QWidget* self = nullptr;
    // layout - the layout for self, managing the contained widgets
    QFormLayout* layout = nullptr;
    // message - a quick way to show a note to the user without relying on the
    //          console output
    QMessageBox* message = nullptr;

public:
    VisGroup(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        self = new QWidget();
        layout = new QFormLayout(self);
        message = new QMessageBox();
        message->setWindowTitle("Refbox UI");
    }

    virtual ~VisGroup();

    virtual QWidget* getWidget() { return self; }
    virtual QLayout* getLayout() { return layout; }

    /*!
       * \brief Disable update of the Widget if hasFocus returns true
       *
       * Any subGroup can disable being updated by returning true.
       * ie when we are on a page that allows the user to create new instances and
       * updating would make the userinput disappear.
       */
    virtual bool hasFocus() = 0;

public Q_SLOTS:
    /*!
       * \brief Causes the Widget to update itself
       *
       * When update is being called the subGroup gets a chance to collect the
       * current data from the worldmodel.
       */
    virtual void update() = 0;
};

/************************************************************************/
/*!
   * \brief The outer Widget managing all subGroups
   *
   * The MainVisGroup has a couple of tasks
   *    - Save the subGroups it gets registered via the registerSubclass-Method
   *    - Provide a ComboBox to select the subGroups
   *    - Provide a 'Refresh' Button to update the currently selected subGroups
   *    - Update subGroups based on WM - Entry-Modified-Callback
   *    - Update subGroups periodically based on Timer
   */
class MainVisGroup : public VisGroup {
    Q_OBJECT
private:
    QComboBox* mainComboBox = nullptr;
    QStackedWidget* mainStackedWidget = nullptr;
    QPushButton* refreshButton = nullptr;
    QHBoxLayout* buttonHBoxLayout = nullptr;
    // QTimer*         timer             = nullptr;

    // save all instances passed in via the registerSubclass-Method
    std::vector<VisGroup*> subGroups;

public:
    MainVisGroup(QWidget* parent = nullptr)
        : VisGroup(parent)
    {
        // Init
        mainComboBox = new QComboBox(self);
        mainStackedWidget = new QStackedWidget(self);
        refreshButton = new QPushButton();
        buttonHBoxLayout = new QHBoxLayout();
        // timer               = new QTimer();
        subGroups = std::vector<VisGroup*>();

        // Draw
        refreshButton->setText("Refresh");
        refreshButton->setMaximumWidth(80);

        buttonHBoxLayout->addWidget(mainComboBox);
        buttonHBoxLayout->addWidget(refreshButton);

        layout->addRow(buttonHBoxLayout);
        layout->addRow(mainStackedWidget);

        // timer->setInterval(30 * 1000);
        // timer->start();

        // Wire
        connect(refreshButton, SIGNAL(clicked()),
            this, SLOT(callUpdate()));
        connect(mainComboBox, SIGNAL(activated(int)),
            this, SLOT(callUpdate(int)));
        connect(mainComboBox, SIGNAL(activated(int)),
            mainStackedWidget, SLOT(setCurrentIndex(int)));
        // connect(timer, SIGNAL(timeout()),
        //         this, SLOT(timer_cb()));
    }

    virtual ~MainVisGroup();

    /*!
       * \brief Reimplementation of the Qt function
       *
       * Updates the UI, usually by getting new data from the Worldmodel and
       *  redrawing the thing.
       */
    virtual void update() override
    { /*this one likes to be empty */
    }

    /*!
       * \brief Hand a VisGroup-SubClass-Instance over to the MainVisGroup
       *
       * This makes the SubClass appear in the RViz Plugin.
       * It also causes the SubClass to be managed by the MainVisGroup.
       */
    void registerSubclass(VisGroup* subGroup, const char* display_name)
    {
        subGroups.push_back(subGroup);
        mainStackedWidget->addWidget(subGroup->getWidget());
        mainComboBox->addItem(display_name);
    }

    /*!
       * \brief Disable update of the Widget if hasFocus returns true
       *
       * Any subGroup can disable being updated by returning true.
       */
    // we would like to know if the user is currently inputting data
    //      ideally this method would return true when the user currently
    //      has a widget selected (via mouse or keyboard)
    // TODO: qt's widget->hasFocus() method does not behave as needed,
    //      returning false even if the user currently has a widget selected
    virtual bool hasFocus() override
    {

        for (auto e : subGroups) {
            if (e->hasFocus()) {
                return true;
            }
        }
        return false;
    }

public Q_SLOTS:
    /*!
       * \brief Update via the Refresh Button
       */
    virtual void callUpdate(int index = -1)
    {
        if (index < 0) {
            index = mainStackedWidget->currentIndex();
        }
        if ((int)subGroups.size() > index) {
            subGroups[index]->update();
            // timer->start();
        }
    }
};

/************************************************************************/
/*!
   * \brief Qt widget displaying worldmodel information at "/World"
   *
   * This widget displays the World-Mapname as readonly
   */
class WorldVisGroup : public VisGroup {
    Q_OBJECT
private:
    QComboBox* worldInstanceCombo = nullptr;
    QLineEdit* mapNameLineEdit = nullptr;

public:
    WorldVisGroup(QWidget* parent = new QWidget())
        : VisGroup(parent)
    {
        // init
        mapNameLineEdit = new QLineEdit();
        worldInstanceCombo = new QComboBox();

        // draw
        layout->addRow("Select World", worldInstanceCombo);
        mapNameLineEdit->setPlaceholderText("Placeholder MapName");
        mapNameLineEdit->setReadOnly(true);
        layout->addRow("Map Name", mapNameLineEdit);

        // update
        this->update();
    }

    virtual ~WorldVisGroup();

    /*!
       * \brief wire connects and disconnect ALL existing Signal-Slots Connections
       *
       * A simple wire() call establishes all connections, a wire(false) call
       * disables all connections. This is usefull, so that when updating the UI
       * there wont be a number of writeFunctions called, "updating" the values,
       * that have just been read from the worldmodel.
       */
    virtual void wire(bool make_connections = true)
    {
        if (make_connections) {
            connect(worldInstanceCombo, SIGNAL(activated(int)),
                this, SLOT(update()));
        } else {
            disconnect(worldInstanceCombo, SIGNAL(activated(int)),
                this, SLOT(update()));
        }
    }
    /*!
       * \brief Disable update of the Widget if hasFocus returns true
       *
       * Any subGroup can disable being updated by returning true.
       * ie when we are on a page that allows the user to create new instances and
       * updating would make the userinput disappear.
       */
    virtual bool hasFocus() override
    {
        // we would like to know if the user is currently inputting data
        //      ideally this method would return true when the user currently
        //      has a widget selected (via mouse or keyboard)
        // TODO: qt's widget->hasFocus() method does not behave as needed,
        //      returning false even if the user currently has a widget selected

        // this VisNode does not have user input
        return false;
    }

public Q_SLOTS:
    /*!
       * \brief Reimplementation of the Qt fucntion - updates the UI
       *
       * Get the up to date informationi from the worldmodel and display it in the
       * UI
       */
    virtual void update() override
    {
        QString state = worldInstanceCombo->currentText();

        // wire
        wire(false);

        // clear current data
        worldInstanceCombo->clear();
        mapNameLineEdit->clear();

        worldInstanceCombo->setCurrentIndex(0);
        mapNameLineEdit->setPlaceholderText("mapName.at(0).c_str()");

        // get fresh data
        // WorldmodelClient& wmc = WorldmodelClient::getInstance();
        // try{
        //     vecInt worlds = wmc.getInts("/World/*/ID");
        //     for (auto e : worlds){
        //         worldInstanceCombo->addItem(QString::number(e));
        //     }
        //     if (state.isEmpty() && worlds.size()){
        //         worldInstanceCombo->setCurrentIndex(0);
        //     } else {
        //         int index = worldInstanceCombo->findText(state);
        //         worldInstanceCombo->setCurrentIndex(index);
        //     }
        //
        //
        //     // no defaults, omit if no world selected
        //     if (worldInstanceCombo->currentIndex() >= 0){
        //         std::string path = "/World/";
        //         path += worldInstanceCombo->currentText().toUtf8().constData();
        //         vecStr mapName = wmc.getStrings(path+"/MapName");
        //         if(mapName.size()){
        //             mapNameLineEdit->setPlaceholderText(mapName.at(0).c_str());
        //         }
        //     }
        // }
        // catch( std::runtime_error e ){
        //     ROS_ERROR_STREAM("[WMP] Failed to update WorldVisGroup");
        // }

        // reconnect
        wire();
    }
};

/************************************************************************/
/*!
   * \brief The Actual RVIZ Plugin
   *
   * This is the class that gets registered an exported as a RVIZ Plugin.
   * It itself provides a stacked widget to show different VisGroups and a
   * refresh Button to update the currently selected VisGruop.
   */
class RefboxUI : public rviz::Panel {
    Q_OBJECT
private:
    MainVisGroup mainVG;
    WorldVisGroup worldVG;

    ros::NodeHandle nh;

public:
    RefboxUI(QWidget* parent = nullptr);
    ~RefboxUI() {}

}; // class Refbox UI

} // ns atwork_refbox_ros
