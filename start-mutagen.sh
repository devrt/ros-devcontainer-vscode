#!/bin/sh

mutagen create \
    --symlink-mode ignore \
    src docker://$(docker-compose ps -q workspace)/workspace/src
