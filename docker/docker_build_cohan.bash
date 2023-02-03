#!/bin/bash

UPAR="--build-arg UID=`id -u` --build-arg GID=`id -g`"

docker build $UPAR -t marrtino:system -f Dockerfile.system . && \
docker build -t marrtino:base -f Dockerfile.base . && \
docker build --no-cache -t marrtino:navigation-cohan -f Dockerfile.navigation-cohan .

