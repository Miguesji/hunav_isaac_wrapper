source /opt/ros/humble/setup.bash
#source $ROS_WORKSPACE/install/setup.bash
#Poner esto en el manual de usuario?

#hunav_evaluator
ros2 run hunav_evaluator hunav_evaluator_node &
#hunav_agent_manager
ros2 run hunav_agent_manager hunav_agent_manager &
wait
