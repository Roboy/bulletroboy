#!/usr/bin/bash

TOPIC="/exoforce/control/initialize_request"
MSGTYPE="roboy_middleware_msgs/InitExoforceRequest"

POSELEFT="{position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 0}}"
POSERIGHT="{position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 0}}"
[INFO] [1615548517.778893250] [operator]: left_wrist init pose: [[0.39584545195102694, 0.14829842746257782, 0.6122045516967773], [0.1263940930366516, 0.8053148984909058, -0.0976242795586586, 0.5709308385848999]]
[INFO] [1615548517.782801323] [operator]: right_wrist init pose: [[0.40152866542339327, -0.17322880029678345, 0.641670823097229], [-0.09424448758363724, 0.7904344797134399, 0.1135825663805008, 0.5945000052452087]]

# Publish init msg to exoforce
ros2 topic pub $TOPIC $MSGTYPE "{ef_name: ['left_hand', 'right_hand'], ef_enabled: [True, True], ef_init_pose: [$POSELEFT, $POSERIGHT]}"
