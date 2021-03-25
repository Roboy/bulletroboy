#!/usr/bin/bash

if [ "$1" == "-h" ]
then
    echo "usage: dummyoperatorforce ef force direction"
    echo
    echo "arguments:"
    printf "  ef\t\tEnd effector ('left'/'right')\n"
    printf "  force\t\tForce to apply\n"
    printf "  direction\tDirection vector ('up'/'down')\n"
    exit 0
fi

TOPIC="/exoforce/simulation/collision"
MSGTYPE="roboy_simulation_msgs/Collision"

# Setting link id for options 'right' and 'left'
if [ "$1" == "right" ]
then 
    LINKID=4 ## right
elif [ "$1" == "left" ]
then 
    LINKID=7 ## left
fi

# Setting normal vector for option 'up' and 'down'
if [ "$3" == "up" ]
then
    NORMAL="{x: 0.0, y: 0.0, z: 1.0}"
elif [ "$3" == "down" ]
then
    NORMAL="{x: 0.0, y: 0.0, z: -1.0}"
fi

# Publish collision msg to ROS2
ros2 topic pub $TOPIC $MSGTYPE "contact_points: [{linkid: $LINKID, normalforce: $2, contactnormal: $NORMAL}]"
