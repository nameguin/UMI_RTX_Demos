#!/bin/bash

current_dir=$(pwd)

sudo -i << EOF

cd "$current_dir"

### sourcing ROS files
source /opt/ros/iron/setup.bash
source ros2_ws/install/setup.bash

### Launch the daemon
# Finds the USB port used for the arm, assumed that only one port is used
if [ -z "$(ls /dev/ttyUSB* 2>/dev/null)" ]
then
  echo "Arm not connected"
else
  cd umi-rtx/bin
  sudo ./rtxd $(ls /dev/ttyUSB* 2>/dev/null)

  echo "------------------"
  ### Launch umi_rtx_controller
  cd ../..
  if [ -e ./umi-rtx/ports/rtx-socket ]; then
  # ros2 run umi_rtx_controller nodeArm
      echo "Start ROS2 interface ..."

      ros2 launch umi_rtx_controller arm.launch.py

      ##### REMOVE NEXT LINE IF YOU WANT LOGS #####
      echo "Delete logs...."
      rm -rf logs/*
      cd
      rm -rf .ros/log/*
      #####

      echo "------------------"
      echo "Reboot ROS2 daemon"
      ros2 daemon stop
      ros2 daemon start
  else 
      echo "umi-rtx/ports/rtx-socket file not found"
  fi
fi

exit
EOF
