# ROS2
source /opt/ros/humble/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh

export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
#export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ai_robotics/ros2_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
export _colcon_cd_root=/opt/ros/humble/
#export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=burger
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

ros2_colb() {
	rosdep install -i --from-path src --rosdistro humble -y
	colcon build --symlink-install
	source install/setup.bash
}

ros2_source() {
	source install/setup.bash
}

ros2_colb_pkg() {
	rosdep install -i --from-path src --rosdistro humble -y
	colcon build --symlink-install --packages-up-to $1
	source install/setup.bash
}

ros2_clean() {
    rm -r ./build
    rm -r ./install
    rm -r ./log
}

alias rosdep='rosdep --rosdistro=humble'

echo "Workspace initialized for ROS2 Humble"
