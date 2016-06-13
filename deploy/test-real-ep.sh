#!/bin/bash
. /opt/ros/indigo/setup.sh
roscore &
python src/rosbridge/src/fetch_proxy.py
