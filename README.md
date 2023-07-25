  # atwork-commander

  [![pipeline status](http://gitlab.robotto.cs.ovgu.de/robotto/remote/atwork_commander/badges/master/pipeline.svg)](http://gitlab.robotto.cs.ovgu.de/robotto/remote/atwork_commander/-/commits/master) 
  [![pipeline status](http://gitlab.robotto.cs.ovgu.de/robotto/remote/atwork_commander/badges/testing/pipeline.svg)](http://gitlab.robotto.cs.ovgu.de/robotto/remote/atwork_commander/-/commits/testing) 

  Complete reimplementation of the [old](https://github.com/robocup-at-work/at_work_central_factory_hub)
  Referee Box (Refbox) for the @Work-League of RoboCup.

  This Refbox is a native ROS application.
  However, it aims to enable multiple communication backends through individual plugins.
  Additionally, a RViz GUI will be developed, which will provide enhanced visualization
  capabilities for referees and visitors / spectators.
  Generation of tasks conforming to the [rule book](https://github.com/robocup-at-work/rulebook)
  are configurable using the ROS parameters in order to be easily adaptable for
  any future changes in rules.


  ## HowTo

  ### Starting the Refbox

  1. Start the __core__ and __com__ components:
  ```
  roslaunch atwork_commander atwork_commander.launch
  ```
  2. Generate a task using the CLI:
  ```
  roslaunch atwork_commander generate.launch task:=<task to generate>
  ```
  3. Wait for robots to register
  4. Start the task execution using the CLI:
  ```
  roslaunch atwork_commander start.launch
  ```

  For testing the Refbox without any robot a fake robot may be used using the **example_robot**:
  `roslaunch atwork_commander example_robot.launch`

  ### Configurations

  - All launch files share the __refbox__ parameter to specify the Refbox to connect
    to. Multiple Refboxes can be started on the same PC using multiple namespaces.
  - When more information is needed, the __verbose__ parameter can be enabled for
    more verbose logging.
  - If a task needs to be sent to only some registered robots, the robots can be
    specified in the __start__ command using the robots parameter in the format
    "<team_name>/<robot_name>"

  ### Runtime control

  - the __forward__ launch file enables manual state change from *PREPARATION* to *EXECUTION*:
  ```
  roslaunch atwork_commander forward.launch
  ```
  - the __stop__ launch file enables stopping of the currently running task:
  ```
  roslaunch atwork_commander stop.launch
  ```

### Easy to use script

You can use the script "default_bringup" which automates the refbox startup.
Its purpose is mainly for new teams so that they can create bagfiles for tasks more easily.
You can also use it for normal refbox startup, however you will need to kill the nodes afterwards manually..
The cleanup is done automatically if you set the  **_immediate** param to **True**

Use e.g. this command to start the script (replace the task for the one you want):

`rosrun atwork_commander default_bringup _task:=BTT1 _immediate:=True _record_rosbag:=True`

NOTE: This requires rosbash to be installed:
`sudo apt install ros-melodic-rosbash ros-melodic-rosbash-params`

You can also modify the script to use other launchfiles than the default ones provided within this package.
This is especially useful for your own arena configurations and robot descriptions.

## Documentation

Currently in the __docu__ folder. Multiple '.graphml' files showing the design
of the architecture and the future GUI (Viewable and editable with e.g.
[yEd Graph Editor](https://www.yworks.com/products/yed))
[Source Code Reference](https://steup.github.io/atwork-commander)

[Issues, Milestones and Releases](https://github.com/robocup-at-work/atwork-commander)

## Sub-Components

The following section will briefly summarize the individual components purpose.
For further information, please have a look at the respective sub-components' README.md.

### [atwork\_commander\_core](atwork_commander_core/README.md)

State-Machine implementation, Pub/Sub and Service implementations to couple all
sub-components.

### [atwork\_commander\_msgs](atwork_commander_msgs/README.md)

Contains ROS messages and service descriptions necessary to communicate within
the Refbox.

### [atwork\_commander\_com](atwork_commander_com/README.md)

Will contain multiple communication plugins to enable flexible communication
links to various types of robots.

### [atwork\_commander\_gui](atwork_commander_gui/README.md)

Aims to enable visualization and control of multiple aspects of a Task before,
during and after a run.

### [atwork\_commander\_gen](atwork_commander_gen/README.md)

Enable dynamic task generation according to specified task types of the
[@Work RuleBook](https://github.com/robocup-at-work/rulebook).

## TODOs

- GUI is currently mockup only
- COM only contains two multi-master ROS communication plugins
- New JurekGen should be fully RuleBook compatible
