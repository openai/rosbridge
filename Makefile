

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
