#!/bin/bash
# 启动Docker容器并在其中运行 ./run_tower.sh
docker run --rm --net=host -v ./OUTPUT_DIR:/root/out -it registry.cn-shanghai.aliyuncs.com/slamibot/fast_lio2:1.11 /bin/bash -c "./run_tower.sh"