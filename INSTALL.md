Build docker image 
```
docker build -t slam_image -f Dockerfile .
```

Build docker container
```
docker run -it --rm --name slam_env --gpus=all --user="$(id -u):$(id -g)" --shm-size 8G --network host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$PWD:/workspace/" slam_image:latest /bin/bash
```

```
docker start slam_env
```
```
xhost +local:$YOUR_CONTAINER_ID
```
```
docker exec -it slam_env /bin/bash
```
```
docker stop slam_env
```
```
xhost -local:$YOUR_CONTAINER_ID
```
# install built opencv
```
cd projects/opencv/build/
```
```
sudo checkinstall
```