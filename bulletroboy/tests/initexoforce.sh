#!/usr/bin/bash

TOPIC="/exoforce/control/initialize_request"
MSGTYPE="roboy_middleware_msgs/InitExoforceRequest"

POSELEFT="{position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 0}}"
POSERIGHT="{position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 0}}"

# Publish init msg to exoforce
ros2 topic pub $TOPIC $MSGTYPE "{ef_name: ['left_hand', 'right_hand'], ef_enabled: [True, True], ef_init_pose: [$POSELEFT, $POSERIGHT]}"
