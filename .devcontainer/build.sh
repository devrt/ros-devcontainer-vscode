#!/bin/sh

for dist in melodic noetic
do
    docker pull ros:$dist
    docker build --build-arg BASE_IMAGE=ros:$dist -f Dockerfile -t devrt/ros-devcontainer-vscode:$dist-desktop .
done