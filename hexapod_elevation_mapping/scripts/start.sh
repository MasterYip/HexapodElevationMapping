#!/bin/bash

# 获取scripts文件夹的绝对路径
SCRIPTS_PATH=$1
PASSWORD=0

# 启动 brige.sh
bash "$SCRIPTS_PATH/brige.sh" &
sleep 2
# # 启动 start_roscore.sh
# bash "$SCRIPTS_PATH/start_roscore.sh" &
sleep 2
# 启动 start_docker.sh
bash "$SCRIPTS_PATH/start_docker.sh"