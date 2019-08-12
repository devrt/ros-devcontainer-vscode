#!/bin/bash -e

USER_ID=$(id -u)
GROUP_ID=$(id -g)

sudo usermod -u $USER_ID -o -m -d /home/developer developer > /dev/null 2>&1
sudo groupmod -g $GROUP_ID developer > /dev/null 2>&1

ln -sfn /workspace /home/developer/workspace

ln -sfn /workspace/.vscode /home/developer/.theia

source /opt/ros/kinetic/setup.bash

exec $@
