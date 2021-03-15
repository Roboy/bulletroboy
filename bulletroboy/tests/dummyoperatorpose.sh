#!/usr/bin/bash

if [ "$1" == "-h" ]
then
    echo "usage: dummyoperatorforce [ef]"
    echo
    echo "arguments:"
    printf "  ef\t\tEnd effector ('left'/'right')\n"
    exit 0
fi

TOPIC="/operator/pose"
MSGTYPE="roboy_middleware_msgs/EFPose"

POSELEFT="{position: {x: -0.14829842746257782, y: 0.6122045516967773, z: 0.39584545195102694}, orientation: {x: 0.0976242795586586, y: 0.1263940930366516, z: -0.8053148984909058, w: 0.5709308385848999}}"
POSERIGHT="{position: {x: 0.17322880029678345, y: 0.641670823097229, z: 0.40152866542339327}, orientation: {x: -0.1135825663805008, y: -0.09424448758363724, z: -0.7904344797134399, w: 0.5945000052452087}}"

EF_NAME="right_hand"
EF_POSE=$POSERIGHT

if [ "$1" == "left" ]
then 
    EF_NAME="left_hand"
    EF_POSE=$POSELEFT
fi


# Publish collision msg to ROS2
ros2 topic pub $TOPIC $MSGTYPE "{ef_name: $EF_NAME, ef_pose: $EF_POSE}"
