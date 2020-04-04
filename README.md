# atwork_commander

Complete reimplementation of a Refery Box (Refbox) for the @Work-League of the RoboCup.

This Refbox is a native ROS application. However it aims to enable multiple communications backends through individual plugins.
Additionally, a RViz GUI will be developed, which provides enhanced visualization capabiliities for referies and visitors / spectators. Generation of tasks conforming to the rule book configurable using the ROS parameter service to be easily adaptable to future changes of the RuleBook.


## HowTo

### Starting the Refbox

1. Start the __core__ and __com__ components: `roslaunch atwork_commander atwork_commander.launch`
2. Generate a task using the CLI: `roslaunch atwork_commander generate.launch task:=<task to generate>`
3. Wait for robots to register
4. Start the task execution using the CLI: `roslaunch atwork_commander start.launch`

For testing the refbox without any robot a fake robot may be used using the **example_robot**:
`roslaunch atwork_commander example_robot.launch`

### Configurations

- All launch files share the __refbox__ parameter to specify the refbox to connect to. Multiple refbox can be started on the same PC using multiple namespaces.
- If more information is wanted the __verbose__ parameter enables more verbose logging.
- If the task shoudl only be send to some registered robots, the robots can be specified in the __start__ command using the robots parameter in the format "<team_name>/<robot_name>"

### Runtime control

- the __forward__ launch file enables manual state change from PREPARATION to EXECUTION:
  `roslaunch atwork_commander forward.launch`
- the __stop__ launch file enables stopping of the currently running task:
  `roslaunch atwork_commander stop.launch`

## Documentation

Currently in the __docu__ folder. Multiple '.graphml' files showing the design of the architecture and the future GUI (Viewable and editable with e.g. [yEd Graph Editor](https://www.yworks.com/products/yed))

## Sub-Components

The following section will briefly summarize the individual components purpose.
For further information have a look at the respective sub-components README.md.

### core

State-Machine implementation and Pub/Sub and Service implementations to couple all sub-components.

### msg

Contains ROS messages and service descriptions necessary to communicate within
the refbox.

### com

Will contain multiple communication plugins to enable flexible communication
links to various types of robots.

### gui

Aims to enable visualization and control of multiple aspects of a Task before, during and after a run.

### gen

Enable dynamic task generation according to specified task types of the @Work
RuleBook.

## TODOs

- GUI is currently mockup only
- COM only contains multi-master ROS communication plugin
- GEN is not fully RuleBook compatible
