#!/bin/bash
set -x
image=$1
if [ x$(REMOTE_DEVBOX) -eq x ]; then
  echo "No REMOTE_DEVBOX in env" 1>&2
  exit 1
fi
shift
exec ssh $(REMOTE_DEVBOX) docker run -i $image $@
