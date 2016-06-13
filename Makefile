

default : docker-push

# Danger: this looks in ~/.mujoco, so you need to have a linux binary and credentials there
EXPERIMENT_EXTRA += $(HOME)/tlbcore ../gym ../mujoco-py ../kluster $(HOME)/.mujoco
DOCKER_EXCLUDES += \
	--exclude 'tlbcore/nodeif' \
	--exclude 'web/d3/test' \

include ../kluster/make-docker.inc

force :

.gitsitchy : force
	( test -e .git && git show -s --pretty=oneline --abbrev-commit && git status -s || /bin/true ) > $@

.gitfiles : force
	( git ls-files -z && printf "\000.gitsitchy" ) >$@

setup-package: force
	rm -rf /tmp/remote-rosbridge
	mkdir -p /tmp/remote-rosbridge
	ssh $(REMOTE_DEVBOX) docker run openai/ros:latest 'bash -c "mkdir -p rosbridge/src && cd rosbridge/src && catkin_create_pkg rosbridge std_msgs control_msgs rospy roscpp >/dev/null && cd .. && tar cf - src"' | tar xf - -C /tmp/remote-rosbridge
	echo "Results in /tmp/remote-rosbridge"

remote-test-real: 	force | remote-build
	ssh -L 9000:127.0.0.1:9000 -t $(REMOTE_DEVBOX) "docker run -p 9000:9000 -ti $(DOCKER_NAME):latest bash -c deploy/test-real-ep.sh"

remote-test-sim: 	force | remote-build
	ssh -L 9000:127.0.0.1:9000 -t $(REMOTE_DEVBOX) "docker run -p 9000:9000 -ti $(DOCKER_NAME):latest bash -x -c deploy/test-sim-ep.sh"

remote-test-shell: 	force | remote-build
	ssh -L 9000:127.0.0.1:9000 -t $(REMOTE_DEVBOX) "docker run -p 9000:9000 -ti $(DOCKER_NAME):latest bash"
