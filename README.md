# Smartcar_ws

## Install dependences

* Only tested on Ubuntu 16.04 LTS + ROS kinectic, I think it is also OK on Ubuntu 18.04 LTS + ROS melodic or higher version.

* ~~If you are on melodic, change "kinectic" to "melodic" in the followings. The same goes for noetic or higher~~.

```bash
sudo apt-get install \
ros-$ROS_DISTRO-joint-state-publisher-gui \
ros-$ROS_DISTRO-controller-manager \
ros-$ROS_DISTRO-gazebo-ros-control \
ros-$ROS_DISTRO-effort-controllers \
ros-$ROS_DISTRO-joint-state-controller \
ros-$ROS_DISTRO-driver-base \
ros-$ROS_DISTRO-rtabmap-ros \
ros-$ROS_DISTRO-ackermann-msgs \
ros-$ROS_DISTRO-teb-local-planner \
ros-$ROS_DISTRO-geographic-msgs ros-$ROS_DISTRO-amcl \
ros-$ROS_DISTRO-gmapping ros-$ROS_DISTRO-map-server \
ros-$ROS_DISTRO-move-base \
ros-$ROS_DISTRO-global-planner
```

## ~~(Optional) Update Gazebo to 7.16~~

```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
sudo apt update
sudo apt upgrade
```
## Build

### Copy model files for gazebo

Copy folders in `src/racecar_gazebo/models/` into `~/.gazebo/models/`.

### Build pakages
```bash
cd ~
git clone https://github.com/ZhanyuGuo/smartcar_ws.git
cd smartcar_ws/
catkin_make
echo "source ~/smartcar_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Run

```bash
roslaunch racecar_gazebo racecar.launch
```
* Use WASD to control the car.

* Input Q to quit.

## Mapping

```bash
roslaunch racecar_gazebo racecar.launch
roslaunch racecar_gazebo slam_gmapping.launch
```

## Save Map

```bash
roscd racecar_gazebo/
cd map/
bash save.bash
```

## Navigation

```bash
roslaunch racecar_gazebo racecar_navigation.launch
```

* Use 2D Pose Estimate to set car's initial pose.

* Use 2D Nav Goal to set car's goal.
