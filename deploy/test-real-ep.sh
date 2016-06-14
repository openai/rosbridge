#!/bin/bash
. /opt/ros/indigo/setup.sh

roscore &
sleep 1

python src/rosbridge/src/fetch_proxy.py
