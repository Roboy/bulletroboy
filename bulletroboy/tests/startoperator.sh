#!/usr/bin/bash

SRV="/operator/control/start"
SRVTYPE="roboy_middleware_msgs/InitExoforce"

POSELEFT="{position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 0}}"
POSERIGHT="{position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 0}}"

# Call start operator service
ros2 service call $SRV $SRVTYPE "{ef_name: ['left_hand', 'right_hand'], ef_enabled: [True, True], ef_init_pose: [$POSELEFT, $POSERIGHT]}"
