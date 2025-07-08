# 1. ROS2 docker setup
Pull docker image of ros2 humble from hub. 
```
$ docker pull osrf/ros:humble-desktop-full\
```
Run docker container based on the image.
```
$docker run -it --rm --privileged \
    -e DISPLAY=$DISPLAY \
    --env="QT_X11_NO_MITSHM=1" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    --hostname $(hostname) \
    --network host \
    --name container_name osrf/ros:humble-desktop bash
```
Enter docker container.
```
$ docker exec -it container_name /bin/bash
```

Enroll source to the bashrc script.
```
$ vim ~/.bashrc

# add below line on the bottom.
source /opt/ros/humble/setup.sh
```
# 2. Clone the repo
```
$ cd ~
$ git clone https://github.com/HYUNHONOH98/ros2_gazebo_lidar_simulate.git
```
# 3. ROS2 plugins setup
## 3.0. basic dependencies
```
$ sudo apt update

$ sudo apt install -y             \
    ros-$ROS_DISTRO-pcl-ros     \
    ros-$ROS_DISTRO-tf2-eigen   \
    ros-$ROS_DISTRO-rviz2       \
    ros-$ROS_DISTRO-xacro


$ sudo apt install -y         \
    build-essential         \
    libeigen3-dev           \
    libjsoncpp-dev          \
    libspdlog-dev           \
    libcurl4-openssl-dev    \
    cmake                   \
    python3-colcon-common-extensions \
    vim

$ sudo apt install \
    ros-humble-gazebo-ros \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-imu-filter-madgwick \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-plugins
```

Install gazebo objects, and world files.
```
$ git clone http://github.com/osrf/gazebo_models
$ git clone https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps
```
Move all the model files to include in the world to `~/.gazebo/models` directory.

## 3.1. jackal
```
$ cd ~/ros2_gazebo_lidar_simulate/ws_jackal
$ colcon build
```
Enroll source to the bashrc script.
```
$ vim ~/.bashrc

# add below line on the bottom.
source ~/ros2_gazebo_lidar_simulate/ws_jackal/install/setup.bash
```

## 3.2. Livox SDK2 (to run fastlio)
```
$ cd ~/ros2_gazebo_lidar_simulate/Livox-SDK2
$ mkdir build
$ cd build
$ cmake .. && make -j
$ sudo make install
```

## 3.3. Livox ros2 driver (to run fastlio)
```
$ cd ~/ros2_gazebo_lidar_simulate/ws_livox/src/livox_ros_driver2
$ source /opt/ros/humble/setup.sh
$ ./build.sh humble
```
Enroll source to the bashrc script.
```
$ vim ~/.bashrc

# add below line on the bottom.
source ~/ros2_gazebo_lidar_simulate/ws_livox/install/setup.bash
```

## 3.4. Fastlio2
Update latest bashrc to the terminal.
```
$ source ~/.bashrc
```
Install fastlio plugin
```
$ cd ~/ros2_gazebo_lidar_simulate/ws_fastlio
$ rosdep install --from-paths src --ignore-src -y
$ colcon build --symlink-install
```

Enroll source to the bashrc script.
```
$ vim ~/.bashrc

# add below line on the bottom.
source ~/ros2_gazebo_lidar_simulate/ws_fastlio/install/setup.bash
```

## 3.5. mid360 lidar simulator on gazebo
```
$ sudo apt install ros-humble-gazebo-ros
```
Install plugin.
```
$ cd ~/ros2_gazebo_lidar_simulate/ws_lidar
$ colcon build
```

Enroll source to the bashrc script.
```
$ vim ~/.bashrc

# add below line on the bottom.
source ~/ros2_gazebo_lidar_simulate/ws_lidar/install/setup.bash
```

# 4. Try launch.
Update the bashrc to the terminal
```
$ source ~/.bashrc
```
Can use shell script to launch jackal simulation with mid360 lidar on it.
```
$ chmod +x ~/ros2_gazebo_lidar_simulate/ws_python/run_gazebo.sh
$ cd ~/ros2_gazebo_lidar_simulate/ws_python/
$ ./run_gazebo.sh
```
Files can be launched indepently too, if the gazebo is working.
