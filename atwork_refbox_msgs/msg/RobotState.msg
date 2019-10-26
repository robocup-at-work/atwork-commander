# RobotState.msg

atwork_refbox_msgs/RobotHeader sender

geometry_msgs/Pose2D pose
nav_msgs/Path path

atwork_refbox_msgs/Object[3] objects_on_robot
atwork_refbox_msgs/Workstation[] current_arena_state



# additional content for season 2021 or later (use markers?)
# TODO: datatype for barriertape
# TODO: datatype for obstacles
