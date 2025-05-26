#!/bin/bash

# Wait for network interfaces if needed
while [ -z "$(ifconfig vxlan 2>/dev/null)" ]; do sleep 1; done

tmux new-session -d -s "color" "source /opt/ws/install/setup.bash && ros2 launch video_compression color_image_raw.launch.py; sleep inf"
tmux new-session -d -s "depth" "source /opt/ws/install/setup.bash && ros2 launch video_compression depth_image_raw.launch.py; sleep inf"

tail -f /dev/null
