
#pragma once

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListView>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QStackedWidget>
#include <QString>
#include <QStringList>
#include <QStringListModel>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include <std_msgs/String.h>

#include "atwork_commander_msgs/ObjectName.h"
#include <atwork_commander/Control.hpp>
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

    std::shared_ptr<atwork_commander::Control> control_ptr;

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

    virtual void setControlPtr(std::shared_ptr<atwork_commander::Control> new_control_ptr)
    {
        if (control_ptr) {
            return;
        }
        control_ptr = new_control_ptr;
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
    QLabel* statusLabel = nullptr;
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
        statusLabel = new QLabel();
        // timer               = new QTimer();
        subGroups = std::vector<VisGroup*>();

        // Draw
        refreshButton->setText("Refresh");
        refreshButton->setMaximumWidth(80);

        buttonHBoxLayout->addWidget(mainComboBox);
        buttonHBoxLayout->addWidget(refreshButton);

        statusLabel->setText("No Connection to Refbox");

        layout->addRow(buttonHBoxLayout);
        layout->addRow(statusLabel);
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
        delete statusLabel;
        // delete timer;
        for (auto e : subGroups) {
            delete e;
        }
        mainComboBox = nullptr;
        mainStackedWidget = nullptr;
        refreshButton = nullptr;
        buttonHBoxLayout = nullptr;
        statusLabel = nullptr;
        // timer             = nullptr;
    }

    virtual void setControlPtr(std::shared_ptr<atwork_commander::Control> new_control_ptr)
    {
        if (control_ptr) {
            return;
        }
        control_ptr = new_control_ptr;

        for (auto e : subGroups) {
            e->setControlPtr(new_control_ptr);
        }
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
    QLabel* taskListLabel = nullptr;
    QComboBox* taskListCombo = nullptr;
    QLabel* prepTimeLabel = nullptr;
    QLineEdit* prepTimeLineEdit = nullptr;
    QLabel* runTimeLabel = nullptr;
    QLineEdit* runTimeLineEdit = nullptr;
    QHBoxLayout* basicHBox = nullptr;

    QPushButton* generateButton = nullptr;
    QPushButton* loadButton = nullptr;
    QHBoxLayout* buttonHBox = nullptr;

    QLabel* arenaStartLabel = nullptr;
    QTextEdit* arenaStartText = nullptr;
    QVBoxLayout* arenaStartVBox = nullptr;
    QLabel* arenaEndLabel = nullptr;
    QTextEdit* arenaEndText = nullptr;
    QVBoxLayout* arenaEndVBox = nullptr;
    QHBoxLayout* arenaHBox = nullptr;

    QLineEdit* pptCavatiesLineEdit = nullptr;

public:
    TaskGenVisGroup(QWidget* parent = new QWidget())
        : VisGroup(parent)
    {
        // init
        taskListLabel = new QLabel();
        taskListCombo = new QComboBox();
        prepTimeLabel = new QLabel();
        prepTimeLineEdit = new QLineEdit();
        runTimeLabel = new QLabel();
        runTimeLineEdit = new QLineEdit();
        basicHBox = new QHBoxLayout();

        generateButton = new QPushButton();
        loadButton = new QPushButton();
        buttonHBox = new QHBoxLayout();

        arenaStartLabel = new QLabel();
        arenaStartText = new QTextEdit();
        arenaStartVBox = new QVBoxLayout();
        arenaEndLabel = new QLabel();
        arenaEndText = new QTextEdit();
        arenaEndVBox = new QVBoxLayout();
        arenaHBox = new QHBoxLayout();

        pptCavatiesLineEdit = new QLineEdit();

        // draw
        taskListLabel->setText("Task Instance: ");
        basicHBox->addWidget(taskListLabel);
        basicHBox->addWidget(taskListCombo);
        prepTimeLabel->setText("Prep-Time: ");
        basicHBox->addWidget(prepTimeLabel);
        prepTimeLineEdit->setReadOnly(true);
        basicHBox->addWidget(prepTimeLineEdit);
        runTimeLabel->setText("Run-Time: ");
        basicHBox->addWidget(runTimeLabel);
        runTimeLineEdit->setReadOnly(true);
        basicHBox->addWidget(runTimeLineEdit);
        layout->addRow(basicHBox);

        generateButton->setText("Generate Task");
        buttonHBox->addWidget(generateButton);
        loadButton->setText("Load Task");
        buttonHBox->addWidget(loadButton);
        layout->addRow(buttonHBox);

        arenaStartLabel->setText("Arena Start State:");
        arenaStartVBox->addWidget(arenaStartLabel);
        arenaStartText->setReadOnly(true);
        arenaStartText->setLineWrapMode(QTextEdit::NoWrap);
        // arenaStartText->setMaximumHeight(16777215);
        arenaStartVBox->addWidget(arenaStartText);
        // arenaStartVBox->setSizeConstraint(QLayout::SetMaximumSize);
        // arenaStartVBox->SetMaximumSize = QSize(16777215, 16777215);
        arenaEndLabel->setText("Arena End State:");
        arenaEndVBox->addWidget(arenaEndLabel);
        arenaEndText->setReadOnly(true);
        arenaEndText->setLineWrapMode(QTextEdit::NoWrap);
        // arenaEndText->setMaximumHeight(16777215);
        arenaEndVBox->addWidget(arenaEndText);
        // arenaEndVBox->setSizeConstraint(QLayout::SetMaximumSize);
        arenaHBox->addLayout(arenaStartVBox);
        arenaHBox->addLayout(arenaEndVBox);
        // arenaHBox->setSizeConstraint(QLayout::SetMaximumSize);
        layout->addRow(arenaHBox);

        pptCavatiesLineEdit->setPlaceholderText(". . . . .");
        pptCavatiesLineEdit->setReadOnly(true);
        layout->addRow("PPT Cavaties:", pptCavatiesLineEdit);

        // update
        this->update();
    }

    virtual ~TaskGenVisGroup()
    {
        delete taskListLabel;
        delete taskListCombo;
        delete prepTimeLabel;
        delete prepTimeLineEdit;
        delete runTimeLabel;
        delete runTimeLineEdit;
        delete basicHBox;
        delete generateButton;
        delete loadButton;
        delete buttonHBox;
        delete arenaStartLabel;
        delete arenaStartText;
        delete arenaStartVBox;
        delete arenaEndLabel;
        delete arenaEndText;
        delete arenaEndVBox;
        delete arenaHBox;
        delete pptCavatiesLineEdit;
        taskListLabel = nullptr;
        taskListCombo = nullptr;
        prepTimeLabel = nullptr;
        prepTimeLineEdit = nullptr;
        runTimeLabel = nullptr;
        runTimeLineEdit = nullptr;
        basicHBox = nullptr;
        generateButton = nullptr;
        loadButton = nullptr;
        buttonHBox = nullptr;
        arenaStartLabel = nullptr;
        arenaStartText = nullptr;
        arenaStartVBox = nullptr;
        arenaEndLabel = nullptr;
        arenaEndText = nullptr;
        arenaEndVBox = nullptr;
        arenaHBox = nullptr;
        pptCavatiesLineEdit = nullptr;
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
            connect(generateButton, SIGNAL(clicked()), this, SLOT(generateTask()));
            connect(loadButton, SIGNAL(clicked()), this, SLOT(update()));
        } else {
            disconnect(taskListCombo, SIGNAL(activated(int)), this, SLOT(update()));
            disconnect(generateButton, SIGNAL(clicked()), this, SLOT(generateTask()));
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

    QString printArenaStateList(std::vector<atwork_commander_msgs::Workstation> ws_list, bool print_empty_container = true)
    {
        using namespace atwork_commander_msgs;

        QString arenaString("");
        const QString indent("    ");
        //workstation (ws), container (c; EMPTY, blue, red, tiles), object (o)
        std::map<std::string, std::map<uint16_t, std::vector<uint16_t>>> target_map;

        std::function<void(int, std::string)> add_line = [&arenaString, &indent](int indents, std::string text) {
            for (int i = 0; i < indents; i++) {
                arenaString.append(indent);
            }
            arenaString.append(QString::fromStdString(text));
            arenaString.append("\n");
        };

        auto is_object = [](uint16_t object) -> bool { return ((object >= Object::ATWORK_START && object < Object::ATWORK_END) || (object >= Object::ROCKIN_START && object < Object::ROCKIN_END)); };
        auto is_container = [](uint16_t container) -> bool { return (container >= Object::CONTAINER_START && container < Object::CONTAINER_END); };

        for (auto& ws : ws_list) {
            for (auto& o : ws.objects) {
                if (is_object(o.object)) {
                    target_map[ws.workstation_name][o.target].push_back(o.object);
                }
                if (is_container(o.object) && print_empty_container) {
                    target_map[ws.workstation_name][o.target].push_back(o.object);
                }
            }
        }

        for (auto ws_it = target_map.begin(); ws_it != target_map.end(); ws_it++) {
            if (ws_it->second.size() == 0) {
                continue;
            }
            add_line(0, "[" + ws_it->first + "]");

            auto ws = ws_it->second;
            if (ws.find(Object::EMPTY) != ws.end()) {
                auto c = ws[Object::EMPTY];
                for (auto& o : c) {
                    if (is_container(o)) { //special for print_empty_container
                        if (ws.find(o) == ws.end()) {
                            add_line(1, std::string("[") + atwork_commander_msgs::objectName(o) + "]");
                        }
                        continue;
                    }
                    add_line(1, atwork_commander_msgs::objectName(o));
                }
            }

            for (auto c_it = ws.begin(); c_it != ws.end(); c_it++) {
                if (c_it->first == Object::EMPTY) {
                    continue;
                }
                add_line(1, std::string("[") + atwork_commander_msgs::objectName(c_it->first) + "]");
                for (auto& o : c_it->second) {
                    add_line(2, atwork_commander_msgs::objectName(o));
                }
            }
        }

        return arenaString;
    }

    //! generate on push
    virtual void generateTask()
    {
        using namespace atwork_commander_msgs;
        Task task;
        try {
            task = control_ptr->generate(taskListCombo->currentText().toStdString());
            ROS_WARN_STREAM(task);
        } catch (const ControlError& e) {
            ROS_ERROR_STREAM("[REFBOX-CONTROL] Error during task generation occured! " << e.what());
            return;
        }

        prepTimeLineEdit->setText(QString::number(task.prep_time.toSec()) + "s");
        runTimeLineEdit->setText(QString::number(task.exec_time.toSec()) + "s");

        QString pptCavatiesString("");
        QString ppt_indent("  ");
        for (auto& w : task.arena_start_state) {
            //TODO PP-Table-Prefix as parameter
            if (w.workstation_name == "PP01" && w.objects.size() > 0) {
                for (auto& o : w.objects) {
                    if (o.object < Object::CAVITY_START || o.object >= Object::CAVITY_END) {
                        continue;
                    }
                    pptCavatiesString.append(atwork_commander_msgs::objectName(o.object));
                    pptCavatiesString.append(ppt_indent);
                }
            }
        }
        pptCavatiesLineEdit->setText(pptCavatiesString);
        arenaStartText->setText(printArenaStateList(task.arena_start_state));
        arenaEndText->setText(printArenaStateList(task.arena_target_state, false));
    }

    /*!
       * \brief Reimplementation of the Qt fucntion - updates the UI
       *
       * Get the up to date informationi from the worldmodel and display it in the
       * UI
       */
    virtual void
    update() override
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
        taskListCombo->addItem(QString("BTT2"));
        taskListCombo->addItem(QString("BTT3"));
        taskListCombo->addItem(QString("RTT"));
        taskListCombo->addItem(QString("PPT"));
        taskListCombo->addItem(QString("FINAL"));

        if (state.isEmpty()) {
            taskListCombo->setCurrentIndex(0);
        } else {
            int index = taskListCombo->findText(state);
            taskListCombo->setCurrentIndex(index);
        }

        prepTimeLineEdit->setText("");
        runTimeLineEdit->setText("");
        arenaStartText->setText("");
        arenaEndText->setText("");

        pptCavatiesLineEdit->setPlaceholderText(". . . . .");

        // reconnect
        wire();
    }
};

/************************************************************************/
/*!
   * \brief Qt widget displaying task generator information
   *
   */
class RobotVisGroup : public VisGroup {
    Q_OBJECT
private:
    QPushButton* playPauseButton = nullptr;
    QHBoxLayout* buttonHBox = nullptr;
    QPushButton* forwardButton = nullptr;
    QPushButton* stopButton = nullptr;

public:
    RobotVisGroup(QWidget* parent = new QWidget())
        : VisGroup(parent)
    {
        // init
        playPauseButton = new QPushButton();
        buttonHBox = new QHBoxLayout();
        forwardButton = new QPushButton();
        stopButton = new QPushButton();

        // draw
        playPauseButton->setText("Play / Pause");
        layout->addRow(playPauseButton);

        forwardButton->setText("Forward");
        buttonHBox->addWidget(forwardButton);
        stopButton->setText("Stop");
        buttonHBox->addWidget(stopButton);
        layout->addRow(buttonHBox);

        // update
        this->update();
    }

    virtual ~RobotVisGroup()
    {
        delete playPauseButton;
        delete buttonHBox;
        delete forwardButton;
        delete stopButton;
        playPauseButton = nullptr;
        buttonHBox = nullptr;
        forwardButton = nullptr;
        stopButton = nullptr;
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
            connect(playPauseButton, SIGNAL(clicked()), this, SLOT(update()));
            connect(forwardButton, SIGNAL(clicked()), this, SLOT(update()));
            connect(stopButton, SIGNAL(clicked()), this, SLOT(update()));
        } else {
            disconnect(playPauseButton, SIGNAL(clicked()), this, SLOT(update()));
            disconnect(forwardButton, SIGNAL(clicked()), this, SLOT(update()));
            disconnect(stopButton, SIGNAL(clicked()), this, SLOT(update()));
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
    virtual void
    update() override
    {
        // wire
        wire(false);

        // // clear current data
        // taskListCombo->clear();
        // pptCavatiesLineEdit->clear();
        //
        // //TODO get list of tasks instances
        // taskListCombo->addItem(QString("BMT"));
        // taskListCombo->addItem(QString("BTT1"));
        // taskListCombo->addItem(QString("BTT2"));
        // taskListCombo->addItem(QString("BTT3"));
        // taskListCombo->addItem(QString("RTT"));
        // taskListCombo->addItem(QString("PPT"));
        // taskListCombo->addItem(QString("FINAL"));
        //
        // if (state.isEmpty()) {
        //     taskListCombo->setCurrentIndex(0);
        // } else {
        //     int index = taskListCombo->findText(state);
        //     taskListCombo->setCurrentIndex(index);
        // }
        //
        // prepTimeLineEdit->setText("");
        // runTimeLineEdit->setText("");
        // arenaStartText->setText("");
        // arenaEndText->setText("");
        //
        // pptCavatiesLineEdit->setPlaceholderText(". . . . .");

        // reconnect
        wire();
    }
};

/************************************************************************/
/*!
   * \brief Qt widget displaying task generator information
   *
   */
class ArenaVisGroup : public VisGroup {
    Q_OBJECT
private:
    // QPushButton* playPauseButton = nullptr;

public:
    ArenaVisGroup(QWidget* parent = new QWidget())
        : VisGroup(parent)
    {
        // init
        // playPauseButton = new QPushButton();

        // draw
        // playPauseButton->setText("Play / Pause");
        // layout->addRow(playPauseButton);

        // update
        this->update();
    }

    virtual ~ArenaVisGroup()
    {
        // delete playPauseButton;
        // playPauseButton = nullptr;
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
            // connect(playPauseButton, SIGNAL(activated(int)), this, SLOT(update()));
            // connect(forwardButton, SIGNAL(clicked()), this, SLOT(update()));
            // connect(stopButton, SIGNAL(clicked()), this, SLOT(update()));
        } else {
            // disconnect(playPauseButton, SIGNAL(activated(int)), this, SLOT(update()));
            // disconnect(forwardButton, SIGNAL(clicked()), this, SLOT(update()));
            // disconnect(stopButton, SIGNAL(clicked()), this, SLOT(update()));
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
    virtual void
    update() override
    {
        // wire
        wire(false);

        // // clear current data
        // taskListCombo->clear();
        // pptCavatiesLineEdit->clear();
        //
        // //TODO get list of tasks instances
        // taskListCombo->addItem(QString("BMT"));
        // taskListCombo->addItem(QString("BTT1"));
        // taskListCombo->addItem(QString("BTT2"));
        // taskListCombo->addItem(QString("BTT3"));
        // taskListCombo->addItem(QString("RTT"));
        // taskListCombo->addItem(QString("PPT"));
        // taskListCombo->addItem(QString("FINAL"));
        //
        // if (state.isEmpty()) {
        //     taskListCombo->setCurrentIndex(0);
        // } else {
        //     int index = taskListCombo->findText(state);
        //     taskListCombo->setCurrentIndex(index);
        // }
        //
        // prepTimeLineEdit->setText("");
        // runTimeLineEdit->setText("");
        // arenaStartText->setText("");
        // arenaEndText->setText("");
        //
        // pptCavatiesLineEdit->setPlaceholderText(". . . . .");

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
    RobotVisGroup robotGenVG;
    ArenaVisGroup arenaGenVG;

    ros::NodeHandle nh;

    std::shared_ptr<atwork_commander::Control> control_ptr;

public:
    RefboxUI(QWidget* parent = nullptr);
    ~RefboxUI() {}

}; // class Refbox UI

} // ns atwork_commander
