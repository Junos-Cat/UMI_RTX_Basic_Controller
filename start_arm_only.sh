#!/bin/bash

current_dir=$(pwd)

#sudo -i << EOF

trap clean SIGINT

# Setup ROS log dir
#export ROS_LOG_DIR=$current_dir/.ros/log
#mkdir -p $ROS_LOG_DIR

#export HOME=$current_dir
#export ROS_LOG_DIR=$HOME/.ros/log
#mkdir -p $ROS_LOG_DIR


# Function to handle clean on Ctrl+C
clean() {
  echo "Stopping all ROS nodes..."
  # Kill background processes
  pkill -P $$
  [ -n "$INV_KIN_PID" ] && kill -9 $INV_KIN_PID 2>/dev/null || true
  [ -n "$NODE_ARM_PID" ] && kill -9 $NODE_ARM_PID 2>/dev/null || true
  # Stop ROS2 daemon (optional)
  ros2 daemon stop
  echo "All nodes stopped."
  exit 0
}

cd "$current_dir"

### sourcing ROS files
#source /opt/ros/foxy/setup.bash
#source /opt/ros/iron/setup.bash
#source ros_control_ws/install/setup.bash
#source ROS_ws/install/setup.bash

### Launch the daemon
# Finds the USB port used for the arm, assumed that only one port is used
if [ -z "$(ls /dev/ttyUSB* 2>/dev/null)" ]; then
  echo "Arm not connected"
  exit 1
else
  echo "Arm connected"

  # To check visually what rtxd programmes are running
  #ps aux | grep rtxd

  echo "Starting daemon"
  cd umi-rtx/bin
  sudo ./rtxd $(ls /dev/ttyUSB* 2>/dev/null) &
  sleep 1

  if pgrep -x rtxd > /dev/null; then
    echo "Daemon is running"

    echo "------------------"

    ### Launch the nodes
    cd ../..
    if [ -e ./umi-rtx/ports/rtx-socket ]; then
      echo "umi-rtx/ports/rtx-socket file has been found"
      
      # Kill any existing nodeArm processes before starting
      echo "Stopping any existing nodeArm nodes..."
      # Find PIDs of all nodeArm processes
      PIDS=$(pgrep -f nodeArm)
      
      if [ -n "$PIDS" ]; then
        echo "Killing PIDs: $PIDS"
        kill -9 $PIDS
        sleep 1  # give time for processes to exit
      else
        echo "No running nodeArm processes found."
      fi
      
      # Kill any existing nodeInverseKinematics processes before starting
      echo "Stopping any existing nodeInverseKinematics nodes..."
      # Find PIDs of all nodeInverseKinematics processes
      PIDS=$(pgrep -f nodeInverseKinematics)
      
      if [ -n "$PIDS" ]; then
        echo "Killing PIDs: $PIDS"
        kill -9 $PIDS
        sleep 1  # give time for processes to exit
      else
        echo "No running nodeInverseKinematics processes found."
      fi

      echo -e "Running nodeArm"

      # Nodes have to be run from umi-rtx
      cd umi-rtx
      ros2 run ros_interface_umi_rtx nodeArm
      # Taking note of PID so the node can be killed later
      NODE_ARM_PID=$!
      echo "nodeArm started with PID $NODE_ARM_PID"
      
      # After control node exits, clean
      clean

    else
      echo "umi-rtx/ports/rtx-socket file not found"
      exit 1
    fi
  else
    echo "Daemon failed to start"
    exit 1
  fi
fi

#EOF

