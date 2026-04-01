sudo docker run -it \
  --gpus all \
  --privileged=true \
  --net=host \
  --shm-size="2g" \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /usr/lib/wsl:/usr/lib/wsl \
  -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all,display \
  -v /root/workspace/xmdog2026/cyberdog_camera:/home/cyberdog_sim/src/cyberdog_camera \
  -v /root/workspace/xmdog2026/cyberdog_controller:/home/cyberdog_sim/src/cyberdog_controller \
  -v /root/workspace/xmdog2026/xmdog_msgs:/home/cyberdog_sim/src/xmdog_msgs \
  innolegend/xmdog_2026:v1