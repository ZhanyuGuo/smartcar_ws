## Install dependences

* Only tested on Ubuntu 16.04 LTS + ROS kinectic, I think it is also OK on Ubuntu 18.04 LTS + ROS melodic.

* If you are on melodic, change "kinectic" to "melodic" in the followings.

```bash
	sudo apt-get install ros-kinetic-joint-state-publisher-gui
	sudo apt-get install ros-kinetic-controller-manager
	sudo apt-get install ros-kinetic-gazebo-ros-control
	sudo apt-get install ros-kinetic-effort-controllers
	sudo apt-get install ros-kinetic-joint-state-controller
	sudo apt-get install ros-kinetic-driver-base
	sudo apt-get install ros-kinetic-rtabmap-ros
	sudo apt-get install ros-kinetic-ackermann-msgs
	sudo apt-get install ros-kinetic-teb-local-planner
	sudo apt-get install ros-kinetic-geographic-msgs
	sudo apt-get install ros-kinetic-amcl
	sudo apt-get install ros-kinetic-gmapping
	sudo apt-get install ros-kinetic-map-server
	sudo apt-get install ros-kinetic-move-base
	sudo apt-get install ros-kinetic-global-planner
```

## Update Gazebo to 7.16

```bash
	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'

	sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
	sudo apt update
	sudo apt upgrade
```
## Buiding

### Copy model files for gazebo

* Copy folders in smart_ws/model_to_gazebo to ~/.gazebo/models.
(construction_cone, start_plane, end_plane, smartcar_plane)

### Build pakages
```bash
	cd ~
	git clone https://gitee.com/guo_zhanyu/smartcar_ws.git
	cd smartcar_ws/src/
	catkin_init_workspace
	cd ..
	catkin_make
	echo "source ~/smartcar_ws/devel/setup.bash" >> ~/.bashrc
	source ~/.bashrc
```

## Running

```bash
	roslaunch racecar_gazebo racecar.launch
```
* Use WASD to control the car.

* Use q to quit.

## Mapping

```bash
	roslaunch racecar_gazebo racecar.launch
	roslaunch racecar_gazebo slam_gmapping.launch
```

## Save Map

```bash
	roscd racecar_gazebo
	cd map
	bash save.bash
```

## Navigation

```bash
	roslaunch racecar_gazebo racecar_navigation.launch
```

* Use 2D Pose Estimate to set car's initial pose.

* Use 2D Nav Goal to set car's goal.
