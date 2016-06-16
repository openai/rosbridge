#!/bin/bash
. /opt/ros/indigo/setup.sh

roscore &
sleep 1

mkdir -p /root/.ros/log
touch /root/.ros/log/FetchRobotGymEnv.log
tail -F /root/.ros/log/*

python src/rosbridge/src/fetch_proxy.py
