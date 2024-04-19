cd ros2_ws
colcon build
source install/setup.sh
ros2 launch umi_rtx_controller simu.launch.py
