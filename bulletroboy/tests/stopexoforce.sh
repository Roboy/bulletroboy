#!/usr/bin/bash

TOPIC="/exoforce/control/stop_request"
MSGTYPE="std_msgs/Empty"

# Publish init msg to exoforce
ros2 topic pub $TOPIC $MSGTYPE "{}"
