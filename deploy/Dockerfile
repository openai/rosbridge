FROM openai/ros:latest

WORKDIR /app

# We first copy just the requirements files, so that we don't have to redo
# this lengthy step if other files have changed.
COPY gym/requirements.txt gym/requirements.txt
RUN pip install -r gym/requirements.txt
#COPY mujoco-py/requirements.txt mujoco-py/requirements.txt
#RUN pip install -r mujoco-py/requirements.txt
COPY rosbridge/deploy/prod_requirements.txt rosbridge/deploy/prod_requirements.txt
RUN pip install -r rosbridge/deploy/prod_requirements.txt

ENV PYTHONPATH=/app:/app/gym:/app/mujoco-py:/app/kluster/lib:/app/tlbcore/common \
  MUJOCO_PY_BUNDLE_PATH=/app/gym/gym/envs/mujoco/mujoco-bundle \
  MUJOCO_PY_MJ_key_path=/app/.mujoco/mjkey.txt \
  MUJOCO_PY_MJPRO_PATH=/app/.mujoco/mjpro131 \
  NODE_PATH=/usr/lib/node_modules:/app

COPY . .

WORKDIR /app/rosbridge
