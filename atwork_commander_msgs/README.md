# atwork_commander_msg

Contains ROS messages and service descriptions necessary to communicate within
the refbox. In case of the native ROS communication plugin, these messages are
also used to communicate with the robots.

## API Structure

The message defining the API of refbox can be seperated into internal and external API components.

- Internal Components
  - Services
    - LoadTask
    - GenerateTask
    - StartTask
    - StateUpdate
    - Arena
  - Messages
    - RefboxState
- External Components
  - Messages
    - RobotState
    - Task
