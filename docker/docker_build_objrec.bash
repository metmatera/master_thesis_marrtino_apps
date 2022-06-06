#!/bin/bash

MACHTYPE=`uname -m`

UPAR="--build-arg UID=`id -u` --build-arg GID=`id -g`"

docker build $UPAR -t marrtino:objrec -f Dockerfile.objrec .

