SIM_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

export GPU_FLAG=(--gpus all)
xhost +local:docker > /dev/null || true

sudo docker run ${GPU_FLAG[@]} \
        -d -ti \
        -e "DISPLAY" \
        -e "QT_X11_NO_MITSHM=1" \
        -e XAUTHORITY \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /etc/localtime:/etc/localtime:ro \
        -v ${SIM_ROOT}/workspace:/workspace \
        --privileged \
        --name "map_exploration" andrewfram/map_exploration:latest \
        > /dev/null
