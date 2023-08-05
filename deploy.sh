#!/usr/bin/env bash
DIR=$(realpath $(dirname "$0"))
rsync -av --exclude '__pycache__' --exclude 'deploy.sh' --exclude '.git' "$DIR" sw8@192.168.2.5: