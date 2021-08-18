
docker build -t slam_image .

# port 8899 for jupyter
# port 5001 for pangolin
# port 12345 for git
docker run -it --name slam_env --gpus=all --shm-size 8G -p 8899:8899 -p 5001:5001 -p 12345:12345 --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" slam_image /bin/bash

docker start slam_env
xhost +local:$YOUR_CONTAINER_ID
docker exec -it slam_env /bin/bash

docker stop slam_env
xhost -local:$YOUR_CONTAINER_ID

# install built opencv
cd projects/opencv/build/
sudo checkinstall