raspicam_node
=============

Groovy ROS node for camera module of Raspberry Pi


Requirements

	- working ROS core and network

	- a Raspberry Pi

	- a camera module 





Get Raspbian

http://elinux.org/RPi_Easy_SD_Card_Setup





Enable Camera and expand FS

http://www.raspberrypi.org/archives/3890



sudo apt-get update

sudo apt-get upgrade


Get ROS Groovy from

http://www.ros.org/wiki/groovy/Installation/Raspbian



sudo apt-get install ros-groovy-image-transport ros-groovy-image-transport-plugins



git clone https://github.com/raspberrypi/userland.git /home/pi/userland

cd /home/pi

mkdir catkin_ws



source /opt/ros/groovy/setup.bash

export ROS_WORKSPACE=/home/pi/catkin_ws



cd /home/pi/catkin_ws

mkdir src

cd src

git clone https://github.com/fpasteau/raspicam_node.git raspicam

cd ..

catkin_make

source devel/setup.bash

then you can run the node using

rosrun raspicam raspicam_node



Topic:

/image/compressed :

	publish sensor_msgs/CompressedImage

	jpeg from the camera module



Services :

/camera/start_capture :

	start video capture and publication



/camera/stop_capture :

	stop video capture and publication (buggy at the moment)



Parameters :

width :

	width of the captured images (0 < width <= 1920)

height : 

	height of the captured images (0 < width <= 1080)

framerate :

	framerate of the captured images (0 < framerate <= 30)

quality :

	quality of the captured images (0 < quality <= 100)



For parameter changes to be applied, the capture need to be restarted using /stop_capture and /start_capture services.





TO DO List :

	- a lot of cleaning and check memory leaks

	- remove warnings from raspicamcontrol

	- fully working /stop_capture

	- reenable vc_gencmd



