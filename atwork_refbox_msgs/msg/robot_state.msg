# robot_state.msg

Header header
geometry_msgs/Pose2D[] pose
geometry_msgs/Point[] points
uint8 another_field
nav_msgs/Path path
atwork_refbox_msgs/object[3] objects_on_robot

# additional content (use markers?)
# TODO: datatype for barriertape
# TODO: datatype for obstacles
