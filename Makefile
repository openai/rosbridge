

default : docker-push

# Danger: this looks in ~/.mujoco, so you need to have a linux binary and credentials there
EXPERIMENT_EXTRA += $(HOME)/tlbcore ../gym ../mujoco-py ../kluster $(HOME)/.mujoco
DOCKER_EXCLUDES += \
	--exclude 'tlbcore/nodeif' \
	--exclude 'web/d3/test' \

include ../kluster/make-docker.inc
include ../kluster/make-utils.inc

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
	ssh -L 6911:127.0.0.1:6911 -t $(REMOTE_DEVBOX) "docker run --privileged -p 6911:6911 -ti $(DOCKER_NAME):latest bash -c deploy/test-real-ep.sh"

remote-test-sim: 	force | remote-build
	ssh -L 6911:127.0.0.1:6911 -t $(REMOTE_DEVBOX) "docker run --privileged -p 6911:6911 -ti $(DOCKER_NAME):latest bash -x -c deploy/test-sim-ep.sh"

remote-test-bg: 	force | remote-build
	ssh -L 6911:127.0.0.1:6911 -t $(REMOTE_DEVBOX) "docker run --privileged -p 6911:6911 -ti $(DOCKER_NAME):latest bash -x -c deploy/test-bg.sh"


remote-test-shell: 	force | remote-build
	ssh -L 6911:127.0.0.1:6911 -t $(REMOTE_DEVBOX) "docker run --privileged -p 6911:6911 -ti $(DOCKER_NAME):latest bash"

.PHONY : rsync-web
docker-rsync.% : .gitfiles .gitsitchy
	rsync -ai --inplace --relative --from0 --files-from .gitfiles -v -e 'bash deploy/docker-rsync.sh' . $*:.

remote-rsync.% : .gitfiles .gitsitchy
	cd $(HOME) && rsync -ai --inplace --relative $(DOCKER_EXCLUDES) $(EXPERIMENT_DIRS_HOMEREL) $*:.
