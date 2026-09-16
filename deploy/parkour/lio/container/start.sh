#!/bin/bash
set -euo pipefail
source /opt/ros/noetic/setup.bash
source /ws/devel/setup.bash
roscore -p 11319 >/tmp/roscore.log 2>&1 &
master=$!
trap 'kill ${mapping:-} "$master" 2>/dev/null || true' EXIT
for attempt in {1..100}; do
    if rosparam list >/dev/null 2>&1; then break; fi
    sleep .1
done
# Exact upstream launch defaults; disable GUI and unbounded PCD accumulation.
cp /ws/src/point_lio_unilidar/launch/mapping_unilidar_l1.launch /tmp/eval.launch
sed -i '/<param name="runtime_pos_log_enable"/a\        <param name="pcd_save/pcd_save_en" type="bool" value="false" />' /tmp/eval.launch
roslaunch /tmp/eval.launch rviz:=false >/tmp/pointlio.log 2>&1 &
mapping=$!
python3 -u /adapter/ros_gateway.py
