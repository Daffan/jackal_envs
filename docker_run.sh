#!/bin/bash
HASH=$(cat /dev/urandom | tr -dc 'a-zA-Z0-9' | fold -w 4 | head -n 1)
name=${USER}_ros_jackal_${HASH}

# Launches a docker container using our image, and runs the provided command

cmd="docker run \
--rm \
-it \
--ipc=host \
--name $name \
--entrypoint="" \
-v `pwd`:/home/jackal_envs_mnt \
zifanxu/ros-jackal:tianshou \
bin/bash ls"
echo $cmd
exec $cmd
