#!/bin/bash

MACHTYPE=`uname -m`

docker build -t marrtino:base -f Dockerfile.base . && \
if [ "$MACHTYPE" = "x86_64" ]; then
  docker build -t iocchi/orazio -f Dockerfile.orazio . && \
  docker build -t iocchi/orazio:2018 -f Dockerfile.orazio2018 .
else
  docker build -t iocchi/orazio:arm64 -f Dockerfile.orazio . && \
  docker build -t iocchi/orazio:2018-arm64 -f Dockerfile.orazio2018 .
fi && \
docker build -t marrtino:teleop -f Dockerfile.teleop . && \
docker build -t marrtino:navigation -f Dockerfile.navigation . && \
docker build -t marrtino:vision -f Dockerfile.vision . && \
docker build -t marrtino:speech -f Dockerfile.speech . 



