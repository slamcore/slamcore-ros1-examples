#!/usr/bin/env bash
set -e
CMD_VEL_TOPIC=$1

if [ -z "$CMD_VEL_TOPIC" ]
then
    echo "Environment variable CMD_VEL_TOPIC is unset. Please set it before running."
    exit 1
fi

# Set velocity to 0 when exiting
trap ctrl_c INT

function ctrl_c() {
    timeout 5 rostopic pub $CMD_VEL_TOPIC geometry_msgs/Twist "
linear:
    x: 0.0
    y: 0.0
    z: 0.0
angular:
    x: 0.0
    y: 0.0
    z: 0.0"
}

echo -e "\033[0;32m [*] Activating software kill-switch...\033[0m"
sleep infinity
