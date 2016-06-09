FROM ubuntu:14.04

# ROS Indigo
# See http://wiki.ros.org/indigo/Installation/Ubuntu
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 0xB01FA116
RUN apt-get update
RUN apt-get install -y ros-indigo-desktop-full
RUN rosdep init
RUN rosdep update
#RUN source /opt/ros/indigo/setup.bash

# Node 5.X
RUN curl -sL https://deb.nodesource.com/setup_5.x | bash -
RUN apt-get update

RUN apt-get install -y nodejs

RUN npm install -g underscore node-gyp jshint mocha uglify-js
RUN npm install -g marked websocket xmldom  eventemitter jquery jsmin2 async codemirror mori cookie
RUN npm install -g hiredis redis mathjax d3 rethinkdb

WORKDIR /app

# We first copy just the requirements file, so that we don't have to redo
# this lengthy step if other files have changed.
COPY rosbridge/deploy/prod_requirements.txt rosbridge/deploy/prod_requirements.txt
RUN apt-get install -y \
    libmagickwand-dev \
    xorg-dev \
    libgl1-mesa-dev \
    xvfb \
    libxinerama1 \
    libxcursor1
RUN apt-get install -y python-pip liblapack-dev gfortran
RUN pip install -r rosbridge/deploy/prod_requirements.txt

ENV PYTHONPATH=/app:/app/gym:/app/mujoco-py:/app/kluster/lib:/app/tlbcore/common \
  MUJOCO_PY_BUNDLE_PATH=/app/gym/gym/envs/mujoco/mujoco-bundle \
  MUJOCO_PY_MJ_key_path=/app/.mujoco/mjkey.txt \
  MUJOCO_PY_MJPRO_PATH=/app/.mujoco/mjpro131 \
  NODE_PATH=/usr/lib/node_modules:/app

COPY . /app/.
RUN ln -sf /app/tlbcore /app/rosbridge/

WORKDIR /app/rosbridge

RUN bash -c '. /opt/ros/indigo/setup.bash && cd src && catkin_init_workspace'
RUN bash -c '. /opt/ros/indigo/setup.bash && catkin_make'
RUN bash -c '. /opt/ros/indigo/setup.bash && . devel/setup.bash'
