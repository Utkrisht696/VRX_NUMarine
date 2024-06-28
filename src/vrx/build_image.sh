#!/bin/bash

# Map host's display socket to docker
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
# DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
# gpu=$(lspci | grep -i '.* vga .* nvidia .*')

# DOCKER_ARGS+=("-e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml")

xhost +local:root

image_name="vrx"
image_tag="latest"
container_name="vrx-container"

# Initialize variables
force_option=false 
clean_option=false
workspace='ros_ws'

# Parse options
while [[ $# -gt 0 ]]; do
  case "$1" in
    --force)
      force_option=true
      shift
      ;;

    --clean)
      clean_option=true
      shift
      ;;
    
    --gz)
      workspace='gz_ws'
      shift
      ;;
      
    *)
      echo "Invalid option: $1"
      exit 1
      ;;
  esac
done


if $force_option; then
  echo "Buidling Existing Docker Image: $image_name"
  docker build -f Dockerfile -t "$image_name:$image_tag" .
  ./build_image.sh  #Comment this line if the script stucks, there might be some issue in the DockerFile

else
  run_command="cd $workspace && source /opt/ros/humble/setup.bash && colcon build --merge-install && exit"

  if $clean_option; then
    run_command="cd $workspace && source /opt/ros/humble/setup.bash && rm -rf devel build && colcon build --merge-install && exit"
    echo "Clean Command Enabled"
  fi


  if docker images --format '{{.Repository}}' | grep -q "$image_name"; then

      echo "Found Docker Image: $image_name:$image_tag"

      echo "Updating the existing Docker image: $image_name:$image_tag"

      docker run -it --rm \
          ${DOCKER_ARGS[@]} \
          -e DISPLAY=$DISPLAY \
          -v $PWD/build/ros_ws:/workspaces/ros_ws/ \
          -v $PWD/gz_packages:/workspaces/gz_ws/src \
          -v $PWD/ros_packages:/workspaces/ros_ws/src \
          -v /var/run/docker.sock:/var/run/docker.sock \
          --name "$container_name" \
          --workdir /workspaces \
          --runtime nvidia \
          --network host \
          $@ \
          "$image_name:$image_tag" \
          bash -c "$run_command"

  else
      echo "Building a new Docker image: $image_name"
      
      docker build -f Dockerfile -t "$image_name:$image_tag" .
      #Comment this line if the script stucks, there might be some issue in the DockerFile
  fi

fi