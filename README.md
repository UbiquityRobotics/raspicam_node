# raspicam_node


Groovy ROS node for camera module of Raspberry Pi

Now works at 90 fps thanks to the new firmware provided by the Raspberry Pi foundation



## Requirements

* working ROS core and network

* a Raspberry Pi

* a camera module 


## Build Intructions

1. Get Raspbian Wheezy from
   [http://elinux.org/RPi_Easy_SD_Card_Setup](http://elinux.org/RPi_Easy_SD_Card_Setup).

2. Enable Camera and expand FS by reading
   [http://www.raspberrypi.org/archives/3890](http://www.raspberrypi.org/archives/3890)

3. Upgrade the system to the latest:

        sudo apt-get update
        sudo apt-get upgrade

4. Install ROS
   [Groovy](http://www.ros.org/wiki/groovy/Installation/Raspbian].

5. Install additional groovy packages:

        sudo apt-get install ros-groovy-image-transport ros-groovy-image-transport-plugins
        sudo apt-get ros-groovy-image-transport-plugins ros-groovy-camera-info-manager

6. Make clone of the Raspberry Pi user land programs source code:

        git clone https://github.com/raspberrypi/userland.git ~/userland

7. Make a catkin workspace:

        cd ~
	mkdir -p catkin_ws/src
        source /opt/ros/groovy/setup.bash
        export ROS_WORKSPACE=~/catkin_ws

8. Install raspicam node:

        cd ~/catkin_ws/src
        #git clone https://github.com/fpasteau/raspicam_node.git raspicam
        git clone https://github.com/UbiquityRobotics/raspicam_node.git raspicam

9. Build catkin workspace:

        cd ~/catkin_ws
        catkin_make

10. Make sure that catkin workspace is visible to ROS:

        source devel/setup.bash

11. Run the raspicam node:

        roscore &
        rosrun raspicam raspicam_node

## More information:

Topics:

* `/camera/compressed`:
        publish sensor_msgs/CompressedImage
        jpeg from the camera module

* `camera/camera_info`:
        publish  sensor_msgs/CameraInfo
        camera info for each frame

Services:

* `/camera/start_capture`:
        start video capture and publication

* `/camera/stop_capture`:
        stop video capture and publication (buggy at the moment)

* `/set_camera_info`:
        set camera information (used for calibration)
        saved in package://raspicam/calibrations/camera.yaml

Parameters:

* `width`:
        width of the captured images (0 < width <= 1920)

* `height`: 
        height of the captured images (0 < width <= 1080)

* `framerate':
        framerate of the captured images (0 < framerate <= 90)

* `quality`:
        quality of the captured images (0 < quality <= 100)

* `tf_prefix`:
        prefix for frame_id

For parameter changes to be applied, the capture need to be restarted
using `/stop_capture` and `/start_capture` services.


Example 1:

        rosrun raspicam raspicam_node
        rosservice call /camera/start_capture
        rosrun image_view image_view image:=/camera/image _image_transport:=compressed

If you want to try 90 fps mode, you'll have to decrease the quality factor.
To try the 90 fps mode :

        rosrun raspicam raspicam_node _framerate:=90 _quality:=10
        rosservice call /camera/start_capture
        rosrun image_view image_view image:=/camera/image _image_transport:=compressed

## TO DO List :

* remove warnings from raspicamcontrol
* reenable vc_gencmd
