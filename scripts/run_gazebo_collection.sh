#!/bin/bash
# ============================================================================
# RS-RBF Gazebo Data Collection — run inside Docker
# ============================================================================
set -e
DURATION=${1:-30}
OUTDIR="/root/catkin_ws/gazebo_results"
mkdir -p "$OUTDIR"

echo "============================================"
echo "  RS-RBF Gazebo Data Collection"
echo "  Duration: ${DURATION}s"
echo "============================================"

# 1. Cleanup
echo "[1/5] Cleaning old processes..."
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rosmaster 2>/dev/null || true
pkill -9 roslaunch 2>/dev/null || true
sleep 2

# 2. Source and launch
echo "[2/5] Starting Gazebo headless + RS-RBF controller..."
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

export ROBOT_TYPE=go2
export DISPLAY=:0
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=true
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe

roslaunch legged_unitree_description go2_sim.launch gui:=false > /tmp/gazebo_launch.log 2>&1 &
LAUNCH_PID=$!

# 3. Wait for controller
echo "[3/5] Waiting for controller..."
MAX_WAIT=90
WAITED=0
READY=0
while [ $WAITED -lt $MAX_WAIT ]; do
  if rostopic list 2>/dev/null | grep -q "/gazebo/link_states"; then
    echo "  Gazebo + Controller ready (${WAITED}s)"
    READY=1
    break
  fi
  sleep 2
  WAITED=$((WAITED + 2))
done

if [ $READY -eq 0 ]; then
  echo "  ERROR: Failed to start"
  kill $LAUNCH_PID 2>/dev/null
  exit 1
fi

# Wait for robot to stabilize in stance
echo "  Stabilizing (5s)..."
sleep 5

# 4. Start recording
echo "[4/5] Recording rosbag..."
BAGFILE="$OUTDIR/gazebo_tracking.bag"
RECORD_TOPICS="/gazebo/link_states /joint_states /clock /cmd_vel"
rosbag record -O "$BAGFILE" $RECORD_TOPICS --duration=${DURATION} > /tmp/rosbag.log 2>&1 &
BAG_PID=$!
sleep 2

# 5. Send figure-eight trajectory
echo "[5/5] Sending figure-eight reference (${DURATION}s)..."
python3 /root/catkin_ws/scripts/send_lemniscate.py --duration $DURATION --rate 20 2>&1

# Wait for rosbag to finish
echo "Waiting for rosbag to finish..."
wait $BAG_PID 2>/dev/null || true

# Cleanup
echo "Cleaning up..."
kill $LAUNCH_PID 2>/dev/null || true
sleep 2
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 rosmaster 2>/dev/null || true
pkill -9 roslaunch 2>/dev/null || true

echo ""
echo "============================================"
echo "  Data saved to: $OUTDIR"
ls -la "$OUTDIR/"
echo "============================================"
