# ros_kr3_demo

# download & build
cd ~
mkdir ~/ws_moveit/
mkdir ~/ws_moveit/src
cd ~/ws_moveit/src
git clone https://github.com/ros-industrial/kuka_experimental.git
git clone https://github.com/ros-industrial/industrial_core.git
git clone https://github.com/pos0637/ros_kr3_demo.git
cd ~/ws_moveit/src/kuka_experimental/kuka_kr3_support/urdf
rosrun xacro xacro --inorder kr3r540.xacro > kr3r540.urdf
cd ~/ws_moveit
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source ~/ws_moveit/devel/setup.bash

# execute
roscore &
roslaunch ros_kr3_demo demo.launch &
rosrun ros_kr3_demo move_kr3.py &
