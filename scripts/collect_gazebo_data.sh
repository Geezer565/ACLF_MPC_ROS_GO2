#!/bin/bash
# ============================================================================
# Gazebo Data Collection Script for RS-RBF Paper Validation
#
# Runs Go2 simulation in headless mode, sends a figure-eight reference,
# records rosbag, and extracts tracking metrics.
#
# Usage: bash collect_gazebo_data.sh [duration_sec=30]
# ============================================================================
set -e

DURATION=${1:-30}
BAG_DIR="/tmp/gazebo_data_$(date +%Y%m%d_%H%M%S)"
RESULTS_DIR="/home/xiaobing/go2_ros1_ws/gazebo_results"

echo "============================================"
echo "  RS-RBF Gazebo Validation Data Collection"
echo "  Duration: ${DURATION}s"
echo "  Bag dir:  ${BAG_DIR}"
echo "============================================"

# 1. Kill any existing processes
echo "[1/5] Cleaning up old processes..."
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rosmaster 2>/dev/null || true
pkill -9 roslaunch 2>/dev/null || true
sleep 2

# 2. Launch Gazebo + controller in background
echo "[2/5] Starting Gazebo headless + controller..."
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

export ROBOT_TYPE=go2
export DISPLAY=:0
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=true
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe

roslaunch legged_unitree_description go2_sim.launch gui:=false &
LAUNCH_PID=$!
echo "  Launch PID: ${LAUNCH_PID}"

# 3. Wait for controller to be ready
echo "[3/5] Waiting for controller to load..."
MAX_WAIT=60
WAITED=0
while [ $WAITED -lt $MAX_WAIT ]; do
  if rostopic list 2>/dev/null | grep -q "/legged_controller/command"; then
    echo "  Controller ready after ${WAITED}s"
    break
  fi
  sleep 2
  WAITED=$((WAITED + 2))
done

if [ $WAITED -ge $MAX_WAIT ]; then
  echo "  ERROR: Controller failed to start within ${MAX_WAIT}s"
  kill $LAUNCH_PID 2>/dev/null
  exit 1
fi

# Wait for robot to stabilize in stance
echo "  Waiting for robot stabilization (5s)..."
sleep 5

# 4. Start rosbag recording
echo "[4/5] Starting rosbag recording..."
TOPICS=(
  /joint_states
  /legged_robot_gait_command/cmd_vel
  /legged_controller/command
  /clock
)
rostopic list 2>/dev/null | grep -E "state|estimate|wrench|force" || true
rosbag record -O "${BAG_DIR}/gazebo_tracking" "${TOPICS[@]}" &
BAG_PID=$!
echo "  Bag PID: ${BAG_PID}"

# 5. Send figure-eight trajectory
echo "[5/5] Running figure-eight trajectory for ${DURATION}s..."
python3 /home/xiaobing/go2_ros1_ws/scripts/send_lemniscate.py --duration $DURATION || true

# Cleanup
echo "Stopping recording..."
kill $BAG_PID 2>/dev/null || true
sleep 2

echo "Stopping simulation..."
kill $LAUNCH_PID 2>/dev/null || true
sleep 2

# Kill all ROS/Gazebo processes
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rosmaster 2>/dev/null || true

echo ""
echo "============================================"
echo "  Data collection complete!"
echo "  Rosbag: ${BAG_DIR}/gazebo_tracking.bag"
echo "============================================"

# Extract CSV for MATLAB/Python processing
echo "Extracting data to CSV..."
cd "${BAG_DIR}"
for topic in "${TOPICS[@]}"; do
  safe_name=$(echo "$topic" | tr '/' '_')
  rostopic echo -b gazebo_tracking.bag -p "$topic" > "${safe_name}.csv" 2>/dev/null || true
done

echo "Files in ${BAG_DIR}:"
ls -la "${BAG_DIR}/"

echo "Done!"
