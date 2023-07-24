# RobotState.msg
RobotHeader sender

geometry_msgs/Pose2D pose
geometry_msgs/PoseStamped[] path

Object[3] objects_on_robot
Workstation[] current_arena_state



# additional content for season 2021 or later (use markers?)
# TODO: datatype for barriertape
# TODO: datatype for obstacles
