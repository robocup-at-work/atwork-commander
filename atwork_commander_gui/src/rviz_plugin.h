
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
#include <QSizePolicy>
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
    std::unique_ptr<QLabel> taskListLabel;
    std::unique_ptr<QComboBox> taskListCombo;
    std::unique_ptr<QLabel> prepTimeLabel;
    std::unique_ptr<QLineEdit> prepTimeLineEdit;
    std::unique_ptr<QLabel> runTimeLabel;
    std::unique_ptr<QLineEdit> runTimeLineEdit;
    std::unique_ptr<QHBoxLayout> basicHBox;

    std::unique_ptr<QPushButton> generateButton;
    std::unique_ptr<QPushButton> loadButton;
    std::unique_ptr<QHBoxLayout> buttonHBox;

    std::unique_ptr<QLabel> arenaStartLabel;
    std::unique_ptr<QTextEdit> arenaStartText;
    std::unique_ptr<QVBoxLayout> arenaStartVBox;
    std::unique_ptr<QLabel> arenaEndLabel;
    std::unique_ptr<QTextEdit> arenaEndText;
    std::unique_ptr<QVBoxLayout> arenaEndVBox;
    std::unique_ptr<QHBoxLayout> arenaHBox;

    std::unique_ptr<QLineEdit> pptCavatiesLineEdit;

public:
    TaskGenVisGroup(QWidget* parent = new QWidget())
        : VisGroup(parent)
    {
        // init
        taskListLabel.reset(new QLabel);
        taskListCombo.reset(new QComboBox);
        prepTimeLabel.reset(new QLabel);
        prepTimeLineEdit.reset(new QLineEdit);
        runTimeLabel.reset(new QLabel);
        runTimeLineEdit.reset(new QLineEdit);
        basicHBox.reset(new QHBoxLayout);

        generateButton.reset(new QPushButton);
        loadButton.reset(new QPushButton);
        buttonHBox.reset(new QHBoxLayout);

        arenaStartLabel.reset(new QLabel);
        arenaStartText.reset(new QTextEdit);
        arenaStartVBox.reset(new QVBoxLayout);
        arenaEndLabel.reset(new QLabel);
        arenaEndText.reset(new QTextEdit);
        arenaEndVBox.reset(new QVBoxLayout);
        arenaHBox.reset(new QHBoxLayout);

        pptCavatiesLineEdit.reset(new QLineEdit);

        // draw
        taskListLabel->setText("Task Instance: ");
        basicHBox->addWidget(taskListLabel.get());
        basicHBox->addWidget(taskListCombo.get());
        prepTimeLabel->setText("Prep-Time: ");
        basicHBox->addWidget(prepTimeLabel.get());
        prepTimeLineEdit->setReadOnly(true);
        basicHBox->addWidget(prepTimeLineEdit.get());
        runTimeLabel->setText("Run-Time: ");
        basicHBox->addWidget(runTimeLabel.get());
        runTimeLineEdit->setReadOnly(true);
        basicHBox->addWidget(runTimeLineEdit.get());
        layout->addRow(basicHBox.get());

        generateButton->setText("Generate Task");
        buttonHBox->addWidget(generateButton.get());
        loadButton->setText("Load Task");
        buttonHBox->addWidget(loadButton.get());
        layout->addRow(buttonHBox.get());

        arenaStartLabel->setText("Arena Start State:");
        arenaStartVBox->addWidget(arenaStartLabel.get());
        arenaStartText->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        arenaStartText->setReadOnly(true);
        arenaStartText->setLineWrapMode(QTextEdit::NoWrap);
        // arenaStartText->setMaximumHeight(16777215);
        arenaStartVBox->addWidget(arenaStartText.get());
        // arenaStartVBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        // arenaStartVBox->setSizeConstraint(QLayout::SetMaximumSize);
        // arenaStartVBox->SetMaximumSize = QSize(16777215, 16777215);
        arenaEndLabel->setText("Arena End State:");
        arenaEndVBox->addWidget(arenaEndLabel.get());
        arenaEndText->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        arenaEndText->setReadOnly(true);
        arenaEndText->setLineWrapMode(QTextEdit::NoWrap);
        // arenaEndText->setMaximumHeight(16777215);
        arenaEndVBox->addWidget(arenaEndText.get());
        // arenaEndVBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        // arenaEndVBox->setSizeConstraint(QLayout::SetMaximumSize);
        arenaHBox->addLayout(arenaStartVBox.get());
        arenaHBox->addLayout(arenaEndVBox.get());
        // arenaHBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        // arenaHBox->setSizeConstraint(QLayout::SetMaximumSize);
        layout->addRow(arenaHBox.get());

        //pptCavatiesLineEdit->setPlaceholderText(". . . . .");
        pptCavatiesLineEdit->setReadOnly(true);
        layout->addRow("PPT Cavaties:", pptCavatiesLineEdit.get());

        // update
        this->update();
    }

    virtual ~TaskGenVisGroup() {}

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
            connect(taskListCombo.get(), SIGNAL(activated(int)), this, SLOT(update()));
            connect(generateButton.get(), SIGNAL(clicked()), this, SLOT(generateTask()));
            connect(loadButton.get(), SIGNAL(clicked()), this, SLOT(update()));
        } else {
            disconnect(taskListCombo.get(), SIGNAL(activated(int)), this, SLOT(update()));
            disconnect(generateButton.get(), SIGNAL(clicked()), this, SLOT(generateTask()));
            disconnect(loadButton.get(), SIGNAL(clicked()), this, SLOT(update()));
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

    //TODO param set_all_targets_to_empty
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
                    target_map[ws.name][o.target].push_back(o.object);
                }
                if (is_container(o.object) && print_empty_container) {
                    target_map[ws.name][o.target].push_back(o.object);
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
            if (w.name == "PP01" && w.objects.size() > 0) {
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
        // wire
        wire(false);

        // task list
        QString state = taskListCombo->currentText();

        taskListCombo->clear();

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

        //task
        //TODO get current task -> control_ptr->state()
        prepTimeLineEdit->setText("");
        runTimeLineEdit->setText("");
        arenaStartText->setText("");
        arenaEndText->setText("");
        pptCavatiesLineEdit->clear();

        // reconnect
        wire();
    }
};

/************************************************************************/
/*!
   * \brief Qt widget displaying robot information and control buttons
   *
   */
class RobotVisGroup : public VisGroup {
    Q_OBJECT
private:
    std::unique_ptr<QPushButton> playPauseButton;
    std::unique_ptr<QHBoxLayout> buttonHBox;
    std::unique_ptr<QPushButton> forwardButton;
    std::unique_ptr<QPushButton> stopButton;

public:
    RobotVisGroup(QWidget* parent = new QWidget())
        : VisGroup(parent)
    {
        // init
        playPauseButton.reset(new QPushButton);
        buttonHBox.reset(new QHBoxLayout);
        forwardButton.reset(new QPushButton);
        stopButton.reset(new QPushButton);

        // draw
        playPauseButton->setText("Play / Pause");
        layout->addRow(playPauseButton.get());

        forwardButton->setText("Forward");
        buttonHBox->addWidget(forwardButton.get());
        stopButton->setText("Stop");
        buttonHBox->addWidget(stopButton.get());
        layout->addRow(buttonHBox.get());

        // update
        this->update();
    }

    virtual ~RobotVisGroup() {}

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
            connect(playPauseButton.get(), SIGNAL(clicked()), this, SLOT(update()));
            connect(forwardButton.get(), SIGNAL(clicked()), this, SLOT(update()));
            connect(stopButton.get(), SIGNAL(clicked()), this, SLOT(update()));
        } else {
            disconnect(playPauseButton.get(), SIGNAL(clicked()), this, SLOT(update()));
            disconnect(forwardButton.get(), SIGNAL(clicked()), this, SLOT(update()));
            disconnect(stopButton.get(), SIGNAL(clicked()), this, SLOT(update()));
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
    std::unique_ptr<QLabel> known_cavities_label;
    std::unique_ptr<QLabel> rtt_label;
    std::unique_ptr<QLabel> ppt_label;
    std::unique_ptr<QLabel> known_objects_label;

public:
    ArenaVisGroup(QWidget* parent = new QWidget())
        : VisGroup(parent)
    {
        // init
        known_cavities_label.reset(new QLabel);
        rtt_label.reset(new QLabel);
        ppt_label.reset(new QLabel);
        known_objects_label.reset(new QLabel);

        // draw layout and fill static elements
        layout->addRow(rtt_label.get());
        layout->addRow(ppt_label.get());
        layout->addRow(known_cavities_label.get());
        layout->addRow(known_objects_label.get());

        // update
        this->update();
    }

    virtual ~ArenaVisGroup() {}

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

        rtt_label->setText("Number of round table(s): 0");
        ppt_label->setText("Number of PP table(s): 1");
        known_cavities_label->setText("Known cavaties: F20_20_H, F20_20_V, R20_H, M20_H, M20_100_H, S40_40_H");
        known_objects_label->setText("Known objects: 2x F20_20_B, R20, 3x S40_40_G, M20, BB");

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
