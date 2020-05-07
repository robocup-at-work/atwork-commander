
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

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>
#endif

namespace atwork_commander {

/*!
   * \brief Abstract Base Class VisGroup
   *
   * This baseclas specifies the interface for subclasses that should work within
   * the RefboxUI and provides a number of conviences (fucntions and members)
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

    virtual ~VisGroup()
    {
        delete self;
        delete layout;
        delete message;
        self = nullptr;
        layout = nullptr;
        message = nullptr;
    }

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

    virtual ~MainVisGroup()
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
   * \brief Qt widget displaying task generator information
   *
   */
class TaskGenVisGroup : public VisGroup {
    Q_OBJECT
private:
    QComboBox* taskListCombo = nullptr;
    QLineEdit* pptCavatiesLineEdit = nullptr;
    QPushButton* generateButton = nullptr;
    QPushButton* loadButton = nullptr;
    QHBoxLayout* buttonHBoxLayout = nullptr;

public:
    TaskGenVisGroup(QWidget* parent = new QWidget())
        : VisGroup(parent)
    {
        // init
        pptCavatiesLineEdit = new QLineEdit();
        taskListCombo = new QComboBox();
        generateButton = new QPushButton();
        loadButton = new QPushButton();
        buttonHBoxLayout = new QHBoxLayout();

        // draw
        layout->addRow("Select Task Instance", taskListCombo);

        generateButton->setText("Generate Task");
        // generateButton->setMaximumWidth(80);
        loadButton->setText("Load Task");
        // loadButton->setMaximumWidth(80);
        buttonHBoxLayout->addWidget(generateButton);
        buttonHBoxLayout->addWidget(loadButton);
        layout->addRow(buttonHBoxLayout);

        pptCavatiesLineEdit->setPlaceholderText(". . . . .");
        pptCavatiesLineEdit->setReadOnly(true);
        layout->addRow("PPT Cavaties:", pptCavatiesLineEdit);

        // update
        this->update();
    }

    virtual ~TaskGenVisGroup()
    {
        delete taskListCombo;
        delete pptCavatiesLineEdit;
        delete generateButton;
        delete loadButton;
        delete buttonHBoxLayout;
        taskListCombo = nullptr;
        pptCavatiesLineEdit = nullptr;
        generateButton = nullptr;
        loadButton = nullptr;
        buttonHBoxLayout = nullptr;
    }

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
            connect(taskListCombo, SIGNAL(activated(int)), this, SLOT(update()));
            connect(generateButton, SIGNAL(clicked()), this, SLOT(update()));
            connect(loadButton, SIGNAL(clicked()), this, SLOT(update()));
        } else {
            disconnect(taskListCombo, SIGNAL(activated(int)), this, SLOT(update()));
            disconnect(generateButton, SIGNAL(clicked()), this, SLOT(update()));
            disconnect(loadButton, SIGNAL(clicked()), this, SLOT(update()));
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
        QString state = taskListCombo->currentText();

        // wire
        wire(false);

        // clear current data
        taskListCombo->clear();
        pptCavatiesLineEdit->clear();

        //TODO get list of tasks instances
        taskListCombo->addItem(QString("BMT"));
        taskListCombo->addItem(QString("BTT1"));
        taskListCombo->addItem(QString("BTTx"));
        taskListCombo->addItem(QString("RTT"));
        taskListCombo->addItem(QString("Final"));

        if (state.isEmpty()) {
            taskListCombo->setCurrentIndex(0);
        } else {
            int index = taskListCombo->findText(state);
            taskListCombo->setCurrentIndex(index);
        }
        pptCavatiesLineEdit->setPlaceholderText(". . . . .");

        // get fresh data
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
    TaskGenVisGroup taskGenVG;

    ros::NodeHandle nh;

public:
    RefboxUI(QWidget* parent = nullptr);
    ~RefboxUI() {}

}; // class Refbox UI

} // ns atwork_commander
