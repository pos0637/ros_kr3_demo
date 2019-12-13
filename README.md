# ros_kr3_demo

# download & build
cd ~</br>
mkdir ~/ws_moveit/</br>
mkdir ~/ws_moveit/src</br>
cd ~/ws_moveit/src</br>
git clone https://github.com/ros-industrial/kuka_experimental.git</br>
git clone https://github.com/ros-industrial/industrial_core.git</br>
git clone https://github.com/pos0637/ros_kr3_demo.git</br>
cd ~/ws_moveit/src/kuka_experimental/kuka_kr3_support/urdf</br>
rosrun xacro xacro --inorder kr3r540.xacro > kr3r540.urdf</br>
cd ~/ws_moveit</br>
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release</br>
catkin build</br>
source ~/ws_moveit/devel/setup.bash</br>

# execute
roscore &</br>
roslaunch ros_kr3_demo demo.launch &</br>
rosrun ros_kr3_demo move_kr3.py &</br>
