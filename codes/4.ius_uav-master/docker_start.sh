#! /usr/bin/env bash

if [ "$#" != 2 ]; then
    echo -e "usage: ./docker_start.sh container_name image_name:tag"
    exit 0
fi

CONTAINER_NAME=$1
CONTAINER_IMAGE=$2

# --runtime=nvidia
# --gpus all
docker run -itd \
           --privileged \
	   --net host \
	   --hostname ${CONTAINER_NAME} \
           --add-host ${CONTAINER_NAME}:127.0.0.1 \
           --name ${CONTAINER_NAME} \
	   -u zzyu \
           -e DISPLAY=${DISPLAY:-:0} \
           -e USER=zzyu \
           -v /home/${USER}:/home/host \
           -v /dev:/dev \
           ${CONTAINER_IMAGE}

