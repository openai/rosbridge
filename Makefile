

default : force

# Danger: this looks in ~/.mujoco, so you need to have a linux binary and credentials there
EXPERIMENT_EXTRA += $(HOME)/tlbcore ../gym ../mujoco-py ../kluster $(HOME)/.mujoco $(HOME)/fetchrobotics $(HOME)/ur
DOCKER_EXCLUDES += \
	--exclude 'web/d3/test' \
	--exclude 'traces' \
	--exclude 'assets'

include ../kluster/make-docker.inc
include ../kluster/make-utils.inc

force :
