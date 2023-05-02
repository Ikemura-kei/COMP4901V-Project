#! /bin/bash
xhost +local:
docker run --privileged --network host -it --pid=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro arena-fsm-ego-planner-updated-9 bash