

default : remote-rsync.20.fetch.sci.openai.org

# Danger: this looks in ~/.mujoco, so you need to have a linux binary and credentials there
EXPERIMENT_EXTRA += $(HOME)/tlbcore ../gym ../mujoco-py ../kluster $(HOME)/.mujoco $(HOME)/fetchrobotics $(HOME)/ur
DOCKER_EXCLUDES += \
	--exclude 'tlbcore/nodeif' \
	--exclude 'web/d3/test' \
	--exclude 'traces' \
	--exclude 'build.*' \
	--exclude 'assets'

include ../kluster/make-docker.inc
include ../kluster/make-utils.inc

force :

.gitsitchy : force
	( test -e .git && git show -s --pretty=oneline --abbrev-commit && git status -s || /bin/true ) > $@

.gitfiles : force
	( git ls-files -z && printf "\000.gitsitchy" ) >$@

remote-rsync.% : .gitfiles .gitsitchy
	cd $(HOME) && rsync -ai --inplace --relative $(DOCKER_EXCLUDES) $(EXPERIMENT_DIRS_HOMEREL) $*:.
