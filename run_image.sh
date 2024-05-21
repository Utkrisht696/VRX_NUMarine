#!/bin/bash


DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")

xhost +local:root

image_name="vrx"
image_tag="latest"
container_name="vrx-container"
domain_id=0
run_command="cd "

if docker ps --format '{{.Names}}' | grep -q "$container_name"; then
    echo "Attaching a new terminal to $container_name"
    docker exec -it "$container_name" /bin/bash
else
    echo "Starting Container: $container_name"
    echo "Image Name: $image_name:$image_tag"
    docker run -it --rm \
        ${DOCKER_ARGS[@]} \
        -e DISPLAY=$DISPLAY \
        -v $PWD/build/gz_ws:/workspaces/gz_ws/ \
        -v $PWD/build/ros_ws:/workspaces/ros_ws/ \
        -v $PWD/gz_packages:/workspaces/gz_ws/src \
        -v $PWD/ros_packages:/workspaces/ros_ws/src \
        -v /var/run/docker.sock:/var/run/docker.sock \
        -v /dev/input:/dev/input --device-cgroup-rule='c 13:* rmw' \
        --name "$container_name" \
        --workdir /workspaces \
        --runtime nvidia \
        --network host \
        $@ \
        "$image_name:$image_tag" \
        /bin/bash
fi
