FROM openai/ros:latest

WORKDIR /app

# We first copy just the requirements file, so that we don't have to redo
# this lengthy step if other files have changed.
COPY rosbridge/deploy/prod_requirements.txt rosbridge/deploy/prod_requirements.txt
RUN pip install -r rosbridge/deploy/prod_requirements.txt

ENV PYTHONPATH=/app:/app/gym:/app/mujoco-py:/app/kluster/lib:/app/tlbcore/common \
  MUJOCO_PY_BUNDLE_PATH=/app/gym/gym/envs/mujoco/mujoco-bundle \
  MUJOCO_PY_MJ_key_path=/app/.mujoco/mjkey.txt \
  MUJOCO_PY_MJPRO_PATH=/app/.mujoco/mjpro131 \
  NODE_PATH=/usr/lib/node_modules:/app

COPY . /app/.

WORKDIR /app/rosbridge

RUN ["/bin/bash", "-c", "echo '. /opt/ros/indigo/setup.bash' >> /root/.bashrc"]

# temp
#RUN ["/bin/bash", "-c", ". /opt/ros/indigo/setup.bash ; cd src && catkin_init_workspace"]
#RUN ["/bin/bash", "-c", "catkin_make"]
