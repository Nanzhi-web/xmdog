sudo docker run -it \
  --gpus all \
  --privileged=true \
  --net=host \
  --shm-size="7g" \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /usr/lib/wsl:/usr/lib/wsl \
  -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all,display \
  -v /home/nanz/workspace/xmdog/cyberdog_camera:/home/cyberdog_sim/src/cyberdog_camera \
  -v /home/nanz/workspace/xmdog/cyberdog_controller:/home/cyberdog_sim/src/cyberdog_controller \
  -v /home/nanz/workspace/xmdog/xmdog_msgs:/home/cyberdog_sim/src/xmdog_msgs \
  innolegend/xmdog_2026:v1