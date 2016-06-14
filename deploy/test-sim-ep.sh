#!/bin/bash
. /opt/ros/indigo/setup.sh

# Secure af
mkdir -p /root/.vnc
printf '\xf0\xe4\x31\x64\xf6\xc2\xe3\x73' >/root/.vnc/passwd
chmod 600 /root/.vnc/passwd

#vncserver :8
#export DISPLAY=:8
sleep 2

roscore &
sleep 1

roslaunch fetch_gazebo simulation.launch gui:=false headless:=true &
roslaunch fetch_moveit_config move_group.launch &
sleep 1

python src/rosbridge/src/fetch_proxy.py &
sleep 1
tail -F --follow=name /root/.ros/log/FetchRobotGymEnv.log &

sleep 3600
