#!/bin/bash

current_dir=$(pwd)

#sudo -i << EOF

# Function to handle cleanup on Ctrl+C
cleanup() {
  echo "Stopping all ROS nodes..."
  # Kill background processes
  pkill -P $$
  kill -9 $INV_KIN_PID
  kill -9 $NODE_ARM_PID
  # Stop ROS2 daemon (optional)
  ros2 daemon stop
  echo "All nodes stopped."
  exit 0
}

cd "$current_dir"

### sourcing ROS files
source /opt/ros/foxy/setup.bash
source /opt/ros/iron/setup.bash
source ros_control_ws/install/setup.bash
source ROS_ws/install/setup.bash

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

      # Ask user which control node to run
      echo "Select a control node to run:"
      echo "1) nodePositionControl1"
      echo "2) nodePositionControl2"
      echo "3) nodeJointControl"
      read -p "Enter choice [1-3]: " choice

      case $choice in
        1|2)
          # Start nodeInverseKinematics in background
          ros2 run ros_interface_umi_rtx nodeInverseKinematics &
          INV_KIN_PID=$!
          echo "nodeInverseKinematics started with PID $INV_KIN_PID"
          ;;
        3)
          echo "Running nodeJointControl"
          ;;
        *)
          echo "Invalid choice"
          cleanup
          ;;
      esac
      
      echo -e "Running nodeArm"

      # Nodes have to be run from umi-rtx
      cd umi-rtx
      ros2 run ros_interface_umi_rtx nodeArm &
      # Taking note of PID so the node can be killed later
      NODE_ARM_PID=$!
      echo "nodeArm started with PID $NODE_ARM_PID"
      
      echo "Starting selected controller"
      # Run selected node interactively in foreground to get keyboard input
      case $choice in
        1)
          ros2 run ros_interface_umi_rtx nodePositionControl1
          ;;
        2)
          ros2 run ros_interface_umi_rtx nodePositionControl2
          ;;
        3)
          ros2 run ros_interface_umi_rtx nodeJointControl
          ;;
      esac

      # After control node exits, cleanup
      cleanup

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

