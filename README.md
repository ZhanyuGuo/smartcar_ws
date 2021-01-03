Install dependences

	sudo apt-get install ros-kinetic-joint-state-publisher-gui
	sudo apt-get install ros-kinetic-controller-manager
	sudo apt-get install ros-kinetic-gazebo-ros-control
	sudo apt-get install ros-kinetic-effort-controllers
	sudo apt-get install ros-kinetic-joint-state-controller
	sudo apt-get install ros-kinetic-driver-base
	sudo apt-get install ros-kinetic-rtabmap-ros
	sudo apt-get install ros-kinetic-ackermann-msgs
	sudo apt-get install ros-kinetic-teb-local-planner

Update Gazebo to 7.16:

	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'

	sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
	sudo apt update
	sudo apt upgrade

Running: 

roslaunch racecar_gazebo racecar.launch

Mapping: 

roslaunch racecar_gazebo racecar.launch
roslaunch racecar_gazebo slam_gmapping.launch

SaveMap: 

roscd racecar_gazebo
cd map
bash save.bash

Navigation: 

roslaunch racecar_gazebo racecar_navigation.launch

